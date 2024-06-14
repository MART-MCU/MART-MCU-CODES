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
extern void Interpolation(const float Data[][2], int size, float x0, float *y0);
extern void DynTorSat(int slip, const float **mu);
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

    // Slippage coefficient Look-Up Table for acceleration model.
    float mu[67][2]  = {{0.000000,0.000000},
                        {0.004000,0.191296},
                        {0.008000,0.379114},
                        {0.012000,0.560289},
                        {0.016000,0.732229},
                        {0.020000,0.893050},
                        {0.024000,1.041608},
                        {0.028000,1.177434},
                        {0.032000,1.300607},
                        {0.036000,1.411611},
                        {0.040000,1.511198},
                        {0.044000,1.600269},
                        {0.048000,1.679787},
                        {0.052000,1.750716},
                        {0.056000,1.813977},
                        {0.060000,1.870425},
                        {0.064000,1.920843},
                        {0.068000,1.965930},
                        {0.072000,2.006311},
                        {0.076000,2.042538},
                        {0.080000,2.075095},
                        {0.100000,2.195991},
                        {0.120000,2.270693},
                        {0.140000,2.318756},
                        {0.160000,2.350731},
                        {0.180000,2.372581},
                        {0.200000,2.387826},
                        {0.220000,2.398633},
                        {0.240000,2.406380},
                        {0.260000,2.411972},
                        {0.280000,2.416017},
                        {0.300000,2.418935},
                        {0.320000,2.421021},
                        {0.340000,2.422485},
                        {0.360000,2.423480},
                        {0.380000,2.424120},
                        {0.400000,2.424489},
                        {0.420000,2.424649},
                        {0.440000,2.424649},
                        {0.460000,2.424525},
                        {0.480000,2.424307},
                        {0.500000,2.424017},
                        {0.520000,2.423671},
                        {0.540000,2.423283},
                        {0.560000,2.422865},
                        {0.580000,2.422424},
                        {0.600000,2.421968},
                        {0.620000,2.421502},
                        {0.640000,2.421030},
                        {0.660000,2.420555},
                        {0.680000,2.420081},
                        {0.700000,2.419609},
                        {0.720000,2.419141},
                        {0.740000,2.418678},
                        {0.760000,2.418221},
                        {0.780000,2.417772},
                        {0.800000,2.417331},
                        {0.820000,2.416897},
                        {0.840000,2.416472},
                        {0.860000,2.416056},
                        {0.880000,2.415648},
                        {0.900000,2.415249},
                        {0.920000,2.414859},
                        {0.940000,2.414478},
                        {0.960000,2.414105},
                        {0.980000,2.413741},
                        {1.000000,2.413385}};
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
