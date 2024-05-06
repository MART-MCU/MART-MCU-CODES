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
#include "math.h"
#include "IQmathLib.h"

//
// Defines
//
#define MSG_DATA_LENGTH_RX 0   // "Don't care" for a Receive mailbox //REVISAR QUE SIGNIFICA ESTE DATA LENGTH
#define MSG_DATA_LENGTH_TX 8

/* ----- READING MAILBOXES -----*/
#define RX_MSG_OBJ_ID1     1   // Use mailbox 1
#define RX_MSG_OBJ_ID2     2   // Use mailbox 2
#define RX_MSG_OBJ_ID3     3   // Use mailbox 3
#define RX_MSG_OBJ_ID4     4   // Use mailbox 4
#define RX_MSG_OBJ_ID5     5   // Use mailbox 5

/* ----- TEXTING MAILBOXES -----*/
#define TX_MSG_OBJ_ID1     6   // Use mailbox 16  // Set Current
#define TX_MSG_OBJ_ID2     7   // Use mailbox 17  // Set Brake Current
#define TX_MSG_OBJ_ID3     8   // Use mailbox 18  // Set Digital Output
#define TX_MSG_OBJ_ID4     9   // Use mailbox 19  // Set Maximum AC Current
#define TX_MSG_OBJ_ID5     10   // Use mailbox 20 // Set Maximum AC Brake Current
#define TX_MSG_OBJ_ID6     11   // Use mailbox 21 // Set Maximum DC Current
#define TX_MSG_OBJ_ID7     12   // Use mailbox 22 // Set Maximum DC Brake Current
#define TX_MSG_OBJ_ID8     13   // Use mailbox 23 // Drive Enable

/* ----- ELECTRICAL PARAMETERTS -----*/
#define LD                 0.000188 // Ld, Lq Motor Model Inductance (H)
#define Rs                 0.01437  // Rs Motor Stator Winding Inductance (Ohms)
#define LAMBDA_F           0.03275  // Lambda_f Rotor Magnetic Flux (mW)
#define P                  3        // Stator Pole Pairs
#define T_MAX              120      // Maximum Motor temperature (ºC)
#define IT_RATE            1.9      // Rate Between Maximum Current & Maximum Torque (A/Nm)
#define MAX_PEAK_POW       60       // Maximum Motor Peak Power (kW)
#define MAX_CONT_POW       35       // Maximum Motor Continuous Power (kW)
#define MAX_PEAK_TOR       100      // Maximum Motor Peak Torque (Nm)
#define MAX_CONT_TOR       52       // Maximum Motor Continuous Torque (Nm)
#define MAX_MOT_RPM        8000     // Maximum Motor Angular Velocity (rpm)

/* ----- TRANSMISSION PARAMETERTS -----*/
#define J_M_CH             0        // Inertia from motor to transmission chain (kg/m^2)
#define B_M_HC             0        // Viscous friction coefficient from motor to transmission chain (Nm·s/rad)
#define J_CH_BR            0        // Inertia from chain to wheel and brake disc (kg/m^2)
#define B_CH_BR            0        // Viscous friction coefficient from chain to wheel and brake disc (Nm·s/rad)
#define CHI                4        // Speed & torque net transmission ratio (p.u.)
#define R                  0.2      // Wheel radio (m)

/* ----- RESISTANCE MODEL PARAMETERTS -----*/
#define M_VEH              234      // Total vehicle mass (kg)
#define GRAV               9.81     // Gravitational acceleration (m/s^2)
#define DIFF_RAT           0.5      // Differential ratio (p.u.)
#define LT                 1.8      // Total longitude between front and rear wheels (m)
#define L1                 0.9      // Longitude between center of gravity (cog) and front wheels (m)
#define H                  0.4      // Center of gravity height (m)
#define RHO                1.25     // Air density at 25ºC and 1 atmospheres (kg/m^3)
#define CX                 0.28     // Lengthwise aerodynamic coefficient
#define AF                 1        // Front vehicle area (m^2)
#define FR                 0.01     // Rolling coefficient


//
// Globals
//
volatile int16_t cpuTimer0IntCount;

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
void initEQEP(void);
void initCPUTimer(void);
void configCPUTimer(uint32_t, float, float);
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
__interrupt void cpuTimer0ISR(void);
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
    // Initialize GPIOs for use as EQEP1A, EQEP1B, and EQEP1I
    //
    GPIO_setPinConfig(GPIO_20_EQEP1A);
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_STD);

    GPIO_setPinConfig(GPIO_21_EQEP1B);
    GPIO_setPadConfig(21, GPIO_PIN_TYPE_STD);

    GPIO_setPinConfig(GPIO_23_EQEP1I);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);

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

    Interrupt_register(INT_CANB0, &canbISR);
    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);

    initCPUTimer();

    CAN_enableGlobalInterrupt(CANB_BASE, CAN_GLOBAL_INT_CANINT0);
    configCPUTimer(CPUTIMER0_BASE, 200000000, 100);


    Interrupt_enable(INT_TIMER0);
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
    initEQEP();
    CPUTimer_startTimer(CPUTIMER0_BASE);

    while(1)
    {
        while(cpuTimer0IntCount==0);
        cpuTimer0IntCount = 0x0000;

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

__interrupt void cpuTimer0ISR(void){
    ++cpuTimer0IntCount;
    ++status1;
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

void initEQEP(void)
{
    //
    // Configure the decoder for quadrature count mode
    //
    EQEP_setDecoderConfig(EQEP1_BASE, (EQEP_CONFIG_1X_RESOLUTION |
                                       EQEP_CONFIG_QUADRATURE |
                                       EQEP_CONFIG_NO_SWAP));

    EQEP_setEmulationMode(EQEP1_BASE, EQEP_EMULATIONMODE_RUNFREE);

    //
    // Configure the position counter to reset on an index event
    //
    EQEP_setPositionCounterConfig(EQEP1_BASE, EQEP_POSITION_RESET_IDX,
                                  0xFFFFFFFF);

    //
    // Enable the unit timer, setting the frequency to 100 Hz
    //
    EQEP_enableUnitTimer(EQEP1_BASE, (DEVICE_SYSCLK_FREQ / 100));

    //
    // Configure the position counter to be latched on a unit time out
    //
    EQEP_setLatchMode(EQEP1_BASE, EQEP_LATCH_UNIT_TIME_OUT);

    //
    // Enable the eQEP1 module
    //
    EQEP_enableModule(EQEP1_BASE);

    //
    // Configure and enable the edge-capture unit. The capture clock divider is
    // SYSCLKOUT/64. The unit-position event divider is QCLK/32.
    //
    EQEP_setCaptureConfig(EQEP1_BASE, EQEP_CAPTURE_CLK_DIV_64,
                          EQEP_UNIT_POS_EVNT_DIV_32);
    EQEP_enableCapture(EQEP1_BASE);
}


void initCPUTimer(void)
{
    //
    // Initialize timer period to maximum
    //
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF);
    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);

    //
    // Make sure timer is stopped
    //
    CPUTimer_stopTimer(CPUTIMER0_BASE);

    //
    // Reload all counter register with period value
    //
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);

    //
    // Reset interrupt counter
    //
    cpuTimer0IntCount = 0;
}

void configCPUTimer(uint32_t cpuTimer, float freq, float period)
{
    uint32_t temp;

    //
    // Initialize timer period:
    //
    temp = (uint32_t)(freq / 1000000 * period);
    CPUTimer_setPeriod(cpuTimer, temp);

    //
    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    //
    CPUTimer_setPreScaler(cpuTimer, 0);

    //
    // Initializes timer control register. The timer is stopped, reloaded,
    // free run disabled, and interrupt enabled.
    // Additionally, the free and soft bits are set
    //
    CPUTimer_stopTimer(cpuTimer);
    CPUTimer_reloadTimerCounter(cpuTimer);
    CPUTimer_setEmulationMode(cpuTimer,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(cpuTimer);

    //
    // Resets interrupt counters for the three cpuTimers
    //
    if (cpuTimer == CPUTIMER0_BASE)
    {
        cpuTimer0IntCount = 0;
    }

}
