// MCU CODE - CAN COMMUNICATION - TORQUE CONTROL - SLIPPAGE CONTROL - STAURATIONS - COOLING CONTROL //

// EDITED BY: PABLO MORA MORENO & PAULA GIL-CEPEDA GÓMEZ

// RELEASE: V1.00 (VX.YY - >> X = 0 Indicates that this release has been tested only in evaluation board <<)

// DATE: 08/03/2024 10:02

// GENERAL NOTES: For configuring CAN module, message objects, interruptions, etc. can_ex4_simple_transmit, can_ex5_simple_recieve, can_ex2_loopback_interrupts
// can_ex3_external_transmit has been used. For configuring CPU timer, timer_ex1_cputimers has been used.
/* This file contains the main code to control the HV inverter & other systems in eMA24RT*/

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "math.h" // Revisar inclusión de librería

//
// Defines
//
#define MSG_DATA_LENGTH_RX 0   // "Don't care" for a Receive mailbox //REVISAR QUE SIGNIFICA ESTE DATA LENGTH
#define MSG_DATA_LENGTH_TX 8

#define RX_MSG_OBJ_ID1     1   // Use mailbox 1
#define RX_MSG_OBJ_ID2     2   // Use mailbox 2
#define RX_MSG_OBJ_ID3     3   // Use mailbox 3
#define RX_MSG_OBJ_ID4     4   // Use mailbox 4
#define RX_MSG_OBJ_ID5     5   // Use mailbox 5

#define TX_MSG_OBJ_ID1     6   // Use mailbox 16 // Set Current
#define TX_MSG_OBJ_ID2     7   // Use mailbox 17 // Set Brake Current
#define TX_MSG_OBJ_ID3     8   // Use mailbox 18 // Set Digital Output
#define TX_MSG_OBJ_ID4     9   // Use mailbox 19 // Set Maximum AC Current
#define TX_MSG_OBJ_ID5     10   // Use mailbox 20 // Set Maximum AC Brake Current
#define TX_MSG_OBJ_ID6     11   // Use mailbox 21 // Set Maximum DC Current
#define TX_MSG_OBJ_ID7     12   // Use mailbox 22 // Set Maximum DC Brake Current
#define TX_MSG_OBJ_ID8     13   // Use mailbox 23 // Drive Enable

#define LD                 0.000188 // Ld, Lq Motor Model Inductance (H)
#define Rs                 0.01437  // Rs Motor Stator Winding Inductance (Ohms)
#define LAMBDA_F           0.03275  // Lambda_f Rotor Magnetic Flux (mW)
#define P                  3        // Stator Pole Pairs
#define T_MAX              120      // Maximum Motor temperature (ºC)
#define IT_RATE            1.9      // Rate Between Maximum Current & Maximum Torque (A/Nm)
#define MAX_PEAK_POW       60       // Maximum Motor Peak Power (kW)
#define MAX_CONT_POW       35       // Maximum Motor Continuoues Power (kW)
#define MAX_PEAK_TOR       100      // Maximum Motor Peak Torque (Nm)
#define MAX_CONT_TOR       52       // Maximum Motor Continuoues Torque (Nm)
#define MAX_MOT_RPM        8000     // Maximum Motor Angular Velocity (rpm)

//
// Globals
//
uint16_t cpuTimer0IntCount;
// Input message arrays
uint8_t rxMsgData1[8];
uint8_t rxMsgData2[8];
uint8_t rxMsgData3[8];
uint8_t rxMsgData4[8];
uint8_t rxMsgData5[8];

volatile uint32_t errorFlag = 0;
float cnt = 0;

int32_t ERPM_rear   = 0;
int32_t cont_w      = 0;
float   DutyCycle   = 0;
float   DCVoltage   = 0;
float   DCCurrent   = 0;
float   ACCurrent   = 0;
float   RPM_rear    = 0;
float   MotTemp     = 0;
float   CtrlTemp    = 0;
float   dCurrent    = 0;
float   qCurrent    = 0;
int     throttle    = 0;
int     brake       = 0;
uint8_t DriveEnable = 0;
uint8_t DigData     = 0;
float estTorque     = 0;
float AclTorque     = 0;
float BrkTorque     = 0;
uint8_t RX_reg      = 0;
float BrkRmsCur     = 0;
float AclRmsCur     = 0;
uint8_t FaultCode   = 0;
float    CANintCont = 0;

uint32_t status1;
uint32_t status2;
//
// Local Routines
//
void configCPUTimer(uint32_t, float, float);
void initCPUTimers(void);
//
// External Routines
//
extern void decodeCanMessageToErpm(const uint8_t *CanMsgData, int32_t *ErpmData, float *RpmData);
extern void decodeCanMessageToDuty(const uint8_t *CanMsgData, float *Dutydata);
extern void decodeCanMessageToDCVoltage(const uint8_t *CanMsgData, float *DCVoltage);
extern void decodeCanMessageToDCCurrent(const uint8_t *CanMsgData, float *DCCurrent);
extern void decodeCanMessageToACCurrent(const uint8_t *CanMsgData, float *ACCurrent);
extern void decodeCanMessageToTEMP(const uint8_t *CanMsgData, float *TEMPERATURE);
extern void decodeCanMessageToDQCurr(const uint8_t *CanMsgData, float *dCurrent, float *qCurrent);
extern void SetupMsgPhaseCurrent(uint8_t *CanMsgData, const float *ArmsCurr);
//
// Function Prototypes
//
__interrupt void canbISR(void);     // Receive interrupt for CAN-B.
interrupt void cpu_timer0_isr(void);
//
// Main
//
void main(void)
{
    //
    // Local Variables
    //
    uint8_t txMsgData1[8];
    uint8_t txMsgData2[8];
    //uint8_t txMsgData3[8];
    //uint8_t txMsgData4[8];
    //uint8_t txMsgData5[8];
    //uint8_t txMsgData6[8];
    //uint8_t txMsgData7[8];
    //uint8_t txMsgData8[8];
    //
    // Initialize device clock and peripherals
    //

    Device_init();

    //
    // Initialize GPIO
    //
    Device_initGPIO();

    //
    // Configure GPIO pins for CANTX/CANRX
    //
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXB);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXB);


    //
    // Initialize the CAN controller
    //
    CAN_initModule(CANB_BASE);

    //
    // Set up the CAN bus bit rate to 500kHz for each module
    // Refer to the Driver Library User Guide for information on how to set
    // tighter timing control. Additionally, consult the device data sheet
    // for more information about the CAN module clocking.
    //
    CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 500000, 16);
    //
    // Enable interrupts on the CAN B peripheral.
    // Enables Int.line0, Error & Status Change interrupts
    //
    CAN_enableInterrupt(CANB_BASE, CAN_INT_IE0 | CAN_INT_ERROR |
                        CAN_INT_STATUS);


    //CAN_setInterruptMux(CANB_BASE, (5 << 1));

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();
    //
    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();
    //
    // Enable Global Interrupt (INTM) and real-time interrupt (DBGM)
    //
    EINT;
    ERTM;
    //
    // ISRs for each CPU Timer interrupt
    //
    Interrupt_register(INT_CANB0, &canbISR);


    CAN_enableGlobalInterrupt(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    Interrupt_enable(INT_CANB0);


    // Initialize the receive message object used for receiving CAN messages.
    // Message Object Parameters:
    //      CAN Module: A or B
    //      Message Object ID Number: Relative Macro
    //      Message Identifier: 0x###
    //      Message Frame: Standard
    //      Message Type: Receive or Transmit
    //      Message ID Mask: 0x0 (REVISAR PARA QUE SIRVE)
    //      Message Object Flags: None
    //      Message Data Length: "Don't care" for a Receive mailbox (REVISAR PARA QUE SIRVE)
    //

    //
    // SETUP MESSAGE OBJECTS FOR READING IN CAN BUS
    //

    CAN_setupMessageObject(CANB_BASE, RX_MSG_OBJ_ID1, 0x401,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                           CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH_RX);
    CAN_setupMessageObject(CANB_BASE, RX_MSG_OBJ_ID2, 0x421,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                           CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH_RX);
    CAN_setupMessageObject(CANB_BASE, RX_MSG_OBJ_ID3, 0x441,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                           CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH_RX);
    CAN_setupMessageObject(CANB_BASE, RX_MSG_OBJ_ID4, 0x461,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                           CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH_RX);
    CAN_setupMessageObject(CANB_BASE, RX_MSG_OBJ_ID5, 0x481,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                           CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH_RX);
    //
    // SETUP MESSAGE OBJECTS FOR TEXTING IN CAN BUS
    //

    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID1, 0x21,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH_TX);

    /*CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID2, 0x41,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID3, 0xE1,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID4, 0x101,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID5, 0x121,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID6, 0x141,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID7, 0x161,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID8, 0x181,
                               CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                               CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);*/


    //
    // Start CAN module B operations
    //
    CAN_startModule(CANB_BASE);


    while(1)
    {
        /*while(cpuTimer0IntCount == 0);
        cpuTimer0IntCount = 0;*/

        //GpioDataRegs.GPASET.bit.GPIO0 = 1;
        cont_w++;

        // Updating receive data
        if (RX_reg & 0x01)
        {
            decodeCanMessageToErpm(rxMsgData1, &ERPM_rear, &RPM_rear);
            decodeCanMessageToDuty(rxMsgData1, &DutyCycle);
            decodeCanMessageToDCVoltage(rxMsgData1, &DCVoltage);
            RX_reg &= ~(0x01);
        }
        if (RX_reg & 0x02)
        {
            decodeCanMessageToDCCurrent(rxMsgData2, &DCCurrent);
            decodeCanMessageToACCurrent(rxMsgData2, &ACCurrent);
            RX_reg &= ~(0x02);
        }
        if (RX_reg & 0x04)
        {
            decodeCanMessageToTEMP(rxMsgData3, &CtrlTemp);
            decodeCanMessageToTEMP(rxMsgData3, &MotTemp);
            FaultCode = rxMsgData3[4];                     // Very important for establishing error status during drive
            RX_reg &= ~(0x04);
        }
        if (RX_reg & 0x08)
        {
            decodeCanMessageToDQCurr(rxMsgData4, &dCurrent, &qCurrent);
            RX_reg &= ~(0x08);
        }
        if (RX_reg & 0x10)
        {
            throttle = (int8_t)rxMsgData5[0];
            brake    = (int8_t)rxMsgData5[1];
            DriveEnable = rxMsgData5[3];
            DigData     = rxMsgData5[2];
        }

        estTorque = (float)0.49125*qCurrent;                                           // Estimated Torque based on dq motor model

        /* For calculating output desired Torque with more accuracy, derrating, control & saturation curves are needed */
        AclTorque = (float)0.01*throttle*MAX_PEAK_TOR;                                 // Calculated Output Torque rated to throttle signal
        /* Is needed to check the ability for controlling d & q currents independently. If we cannot support this control, Torque-Current curves are needed */
        /* A really conflictive control is flux weakening control, where rotor magnetic flux has to be decreased to afford higher velocities*/
        AclRmsCur = (float)IT_RATE*AclTorque;                                          // Target Phase Current based on

        /*SetupMsgPhaseCurrent(txMsgData1, &AclRmsCur);
        CAN_sendMessage(CANB_BASE, TX_MSG_OBJ_ID1, MSG_DATA_LENGTH_TX, txMsgData1);

        BrkTorque = (float)0.01*brake*MAX_PEAK_TOR;                                 // Calculated Output Torque rated to Brake signal
        BrkRmsCur = (float)IT_RATE*BrkTorque;

        SetupMsgPhaseCurrent(txMsgData1, &BrkRmsCur);
        //CAN_sendMessage(CANB_BASE, TX_MSG_OBJ_ID2, MSG_DATA_LENGTH_TX, txMsgData2);*/
    }
}

//
// Interrupt Service Routines
//

/*canbISR(void)
{
    uint32_t status;

    //
    // Read the CAN-B interrupt status (in the CAN_INT register) to find the
    // cause of the interrupt
    //
    status = CAN_getInterruptCause(CANB_BASE);

    //
    // If the cause is a controller status interrupt, then get the status.
    // During first iteration of every ISR execution, status = 0x8000,
    // which simply means CAN_ES != 0x07.
    //
    if(status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        status = CAN_getStatus(CANB_BASE);  // Return CAN_ES value.
        //
        // Now status = 0x00000010, indicating RxOK.
        //

        //
        // Check to see if an error occurred.
        //
        if(((status  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_MSK) &&
           ((status  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_NONE))
        {
            //
            // Set a flag to indicate some errors may have occurred.
            //
            errorFlag = 1;
        }
    }
    //
    // Check if the cause is the CAN-B receive message object 1. Will be skipped
    // in the first iteration of every ISR execution
    //

    else if(status == RX_MSG_OBJ_ID1)
        {
            //
            // Get the received message
            //
            CAN_readMessage(CANB_BASE, RX_MSG_OBJ_ID1, rxMsgData);

            //
            // Getting to this point means that the RX interrupt occurred on
            // message object 1, and the message RX is complete.  Clear the
            // message object interrupt.
            //
            CAN_clearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID1);

            //
            // Increment a counter to keep track of how many messages have been
            // received. In a real application this could be used to set flags to
            // indicate when a message is received.
            //
            rxMsgCount++;

            //
            // Since the message was received, clear any error flags.
            //
            errorFlag = 0;
        }
    //
    // If something unexpected caused the interrupt, this would handle it.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }

    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CAN_clearGlobalInterruptStatus(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}*/
void canbISR(void){
    ++CANintCont;
    status1 = CAN_getInterruptCause(CANB_BASE);
    switch(status1) {
            case CAN_INT_INT0ID_STATUS:
                status2 = CAN_getStatus(CANB_BASE);

                if(((status2  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_MSK) &&
                   ((status2  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_NONE))
                {
                    errorFlag = 1;
                }
                break;
            case RX_MSG_OBJ_ID1:
                CAN_readMessage(CANB_BASE, RX_MSG_OBJ_ID1, rxMsgData1);
                CAN_clearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID1);
                errorFlag = 0;
                RX_reg |= 0x01;
                break;
            case RX_MSG_OBJ_ID2:
                CAN_readMessage(CANB_BASE, RX_MSG_OBJ_ID2, rxMsgData2);
                CAN_clearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID2);
                errorFlag = 0;
                RX_reg |= 0x02;
                break;
            case RX_MSG_OBJ_ID3:
                CAN_readMessage(CANB_BASE, RX_MSG_OBJ_ID3, rxMsgData3);
                CAN_clearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID3);
                errorFlag = 0;
                RX_reg |= 0x04;
                break;
            case RX_MSG_OBJ_ID4:
                CAN_readMessage(CANB_BASE, RX_MSG_OBJ_ID4, rxMsgData4);
                CAN_clearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID4);
                errorFlag = 0;
                RX_reg |= 0x08;
                break;
            case RX_MSG_OBJ_ID5:
                CAN_readMessage(CANB_BASE, RX_MSG_OBJ_ID5, rxMsgData5);
                CAN_clearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID5);
                errorFlag = 0;
                RX_reg |= 0x10;
                break;
            default:
                break;
        }
    CAN_clearGlobalInterruptStatus(CANB_BASE, CAN_GLOBAL_INT_CANINT0);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}


/*void SetupEQEP(void)
{

}*/
