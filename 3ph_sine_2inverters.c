//#############################################################################
//
// FILE:   empty_driverlib_main.c
//
// TITLE:  Empty Project
//
// Empty Project Example
//
// This example is an empty project setup for Driverlib development.
//
//#############################################################################
// $TI Release: F2837xD Support Library v3.01.00.00 $
// $Release Date: Mon May 22 15:43:40 CDT 2017 $
// $Copyright:
// Copyright (C) 2013-2017 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "math.h"
#include "IQmathLib.h"
#include "eqep_ex2_calculation.h"

//
// Defines
//
// these are for ADC
#define EX_ADC_RESOLUTION   ADC_RESOLUTION_12BIT    // or ADC_RESOLUTION_16BIT
#define EX_ADC_SIGNAL_MODE  ADC_MODE_SINGLE_ENDED   // or ADC_MODE_DIFFERENTIAL

// these are for generating PWM
#define TB_CLK  DEVICE_SYSCLK_FREQ / 2              // Time base clock is SYCLK / 2
#define PWM_CLK 5000                                // We want to output at 5 kHz

#define PRD_VAL (TB_CLK / (PWM_CLK * 2))            // Calculate period value for up-down count mode

#define EPWM1_TIMER_TBPRD   2000U
#define EPWM1_CMP           1000U

#define EPWM2_TIMER_TBPRD   2000U
#define EPWM2_CMP           1000U

#define EPWM3_TIMER_TBPRD   2000U
#define EPWM3_CMP           1000U

#define EPWM4_TIMER_TBPRD   2000U
#define EPWM4_CMP           1000U

#define EPWM5_TIMER_TBPRD   2000U
#define EPWM5_CMP           1000U

#define EPWM6_TIMER_TBPRD   2000U
#define EPWM6_CMP           1000U

#define SHIFT_PHASE         1333U
#define DEAD_BAND           50U


// these are for lookup tables
#define CPUFREQ_MHZ           200
#define PI                    3.14159265

// These are for calculating the speed and mech position
// .9999 / 4000 converted to IQ26 fixed point format
#define MECH_SCALER     16776
// 2 pole pairs in this example
#define POLE_PAIRS      5
// Angular offset between encoder and Phase A
#define CAL_ANGLE       0
// See Equation 5 in eqep_ex2_calculation.c
#define SPEED_SCALER    ((((uint64_t)32 * DEVICE_SYSCLK_FREQ / 64) * 60) / (24000000))
// Base/max rpm is 6000rpm
#define BASE_RPM        6000


// These are for motor parameters
#define L 0.007 // inductance in Henry
#define R 0.5  // per phase resistance in ohms
#define P 5.0 //number of pole pairs
#define Tr 13.3  //rated torque
#define I_m 15.5  // rated current peak
#define v_m 2400
float lambda_m = Tr*2/(P*3.0*I_m) ;
// motor parameters
float speedrpm;
float omega;
float omega2;
float lambda_m2;
float R2;
float L2;

//
// GLOBALS
//
float adcResult_Vdd;
float Tref;
float adcResult_A_v_L1;
float adcResult_A_v_L2;
float adcResult_B_v_L1;
float adcResult_B_v_L2;
float adcResult_C_v_L1;
float adcResult_C_v_L2;

float adcResult_A_i_L1;
float adcResult_A_i_L2;
float adcResult_B_i_L1;
float adcResult_B_i_L2;
float adcResult_C_i_L1;
float adcResult_C_i_L2;


uint16_t speedloopcount = 0;
uint16_t Mainloopcount = 0;
uint16_t EPWM1loopcount = 0;
uint16_t EPWM2loopcount = 0;
uint16_t CMP = 1000U;
uint16_t CMPA = 1000U;
uint16_t CMPB = 1000U;
uint16_t CMPC = 1000U;

// these are for eQEP
//uint16_t enc_pos;
//uint16_t max_pos = 3999;
float max_pos = 3999;
float enc_pos ;

PosSpeed_Object posSpeed =
{
    0, 0, 0, 0,     // Initialize outputs to zero
    MECH_SCALER,    // mechScaler
    POLE_PAIRS,     // polePairs
    CAL_ANGLE,      // calAngle
    SPEED_SCALER,   // speedScaler
    0,              // Initialize output to zero
    BASE_RPM,       // baseRPM
    0, 0, 0, 0      // Initialize outputs to zero
};

uint16_t interruptCount = 0;


// these are for reference currents
float i_aR;
float i_bR;
float i_cR;

// these are estimated voltages
float v_aR;
float v_bR;
float v_cR;


float Imag = 1;
float Imag2;
float theta;
float It ;
float Itlim;
float Vlim;

float gamma = 0;
float Iband = 10;
float phase_offset = 0.11*PI;  // to rectify encoder offset with respect to phase A


// these are for measured currents
float i_a = 0;
float i_b = 0;
float i_c = 0;

// these are for measured voltages
float v_a = 0;
float v_b = 0;
float v_c = 0;


// these are for DAC
float dacVal = 2048;

//
// PID variables
//

typedef struct
{
    double dState;  // Last position input
    double iState;  // integrator state
    double iMax, iMin;  // Maximum and minimum allowable integrator state

    double iGain,   // integral gain
           pGain,   // proportional gain
           dGain;   // derivative gain
} SPid;

float errorA = 0;
float errorB = 0;
float errorC = 0;

SPid PidA;
SPid PidB;
SPid PidC;
SPid Pidspeed;

//
//Function Prototypes
//

// these are for eQEP
void initEQEP(void);

// these are for inverter SPI communication
/*void initSPIAMaster(void);
void configSPIGPIOs(void);
__interrupt void spiaTxFIFOISR(void);
__interrupt void spiaRxFIFOISR(void);
__interrupt void adcA1ISR(void);
__interrupt void adcB1ISR(void);
__interrupt void adcC1ISR(void);*/
// these are for DAC
void configureDAC(void);

//
//Function Prototypes
//
// these are for adc
void initADCs(void);
void initADCSOCs(void);

// these are for EPWM
void initEPWM1(void);
void initEPWM2(void);
void initEPWM3(void);
void initEPWM4(void);
void initEPWM5(void);
void initEPWM6(void);

__interrupt void epwm1ISR(void);
__interrupt void epwm2ISR(void);
__interrupt void epwm3ISR(void);
/*__interrupt void epwm4ISR(void);
__interrupt void epwm5ISR(void);
__interrupt void epwm6ISR(void);*/


// these are for PID
double UpdatePID(SPid * pid, double error, double position) ;

//
// Main
//
void main(void)
{
    //
    // Initiate device clock and periperals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.u
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    //Initialize the PIE vector table with pointers to the shell interrupt Interrupt Service Routines (ISR)
    //
    Interrupt_initVectorTable();

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
    // Configure GPIO0/1, GPIO2/3  and GPIO4/5 as ePWM1A/1B, ePWM2A/2B and ePWM3A/3B
    //
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_1_EPWM1B);

    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_2_EPWM2A);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_3_EPWM2B);

    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_4_EPWM3A);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_5_EPWM3B);

    //
    // Configure GPIO6/7, GPIO8/9 and GPIO10/11 as ePWM4A/B, ePWM5A/B and ePWM6A/B
    //
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_6_EPWM4A);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_7_EPWM4B);

    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_8_EPWM5A);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_9_EPWM5B);

    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_10_EPWM6A);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_11_EPWM6B);

    //
    // Enable an GPIO output on GPIO124 to clear the ENGATE on booster pack
    //
    GPIO_setPadConfig(124, GPIO_PIN_TYPE_PULLUP);   // Enable pullup on GPIO6
    GPIO_writePin(124, 1);                          // Load output latch
    GPIO_setPinConfig(GPIO_124_GPIO124);              // GPIO124 = GPIO124
    GPIO_setDirectionMode(124, GPIO_DIR_MODE_OUT);  // GPIO124 = output
    DEVICE_DELAY_US(10000);

    //
    // Enable an GPIO output on GPIO26 to clear the ENGATE on booster pack
    //
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_PULLUP);   // Enable pullup on GPIO6
    GPIO_writePin(26, 1);                          // Load output latch
    GPIO_setPinConfig(GPIO_26_GPIO26);              // GPIO124 = GPIO124
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_OUT);  // GPIO124 = output
    DEVICE_DELAY_US(10000);


    //
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file
    //
    /*Interrupt_register(INT_SPIA_TX, &spiaTxFIFOISR);
    Interrupt_register(INT_SPIA_RX, &spiaRxFIFOISR);
    Interrupt_register(INT_ADCA1, &adcA1ISR);
    Interrupt_register(INT_ADCB1, &adcB1ISR);
    Interrupt_register(INT_ADCC1, &adcC1ISR);*/
    Interrupt_register(INT_EPWM1, &epwm1ISR);
    Interrupt_register(INT_EPWM2, &epwm2ISR);
    Interrupt_register(INT_EPWM3, &epwm3ISR);
    /*Interrupt_register(INT_EPWM4, &epwm4ISR);
    Interrupt_register(INT_EPWM5, &epwm5ISR);
    Interrupt_register(INT_EPWM6, &epwm6ISR);*/

    //
    // Set up ADCs
    //
    initADCs();
    //
    // Disable sync(Freeze clock to PWM as well)
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    initEPWM1();
    initEPWM2();
    initEPWM3();
    initEPWM4();
    initEPWM5();
    initEPWM6();
    //
    // Enable sync and clock to PWM
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Initializing the SOCs to be triggered by software and ADC interrupts
    initADCSOCs();

    //
    // Setup eQEP1, configuring the unit timer and quadrature capture units
    //
    initEQEP();

    //
    // Configure the DAC module
    //
    configureDAC();

    //
    // Enable interrupts
    //
    Interrupt_enable(INT_EPWM1);
    Interrupt_enable(INT_EPWM2);
    Interrupt_enable(INT_EPWM3);
    /*Interrupt_enable(INT_EPWM4);
    Interrupt_enable(INT_EPWM5);
    Interrupt_enable(INT_EPWM6);*/

    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT ;
    ERTM ;

    //
    // Initialize Pid variables
    //

    PidA.iMax = 2000U;
    PidA.iMin = 0U;
    PidA.pGain = 10U;
    PidA.iGain = 1U;

    PidB.iMax = 2000U;
    PidB.iMin = 0U;
    PidB.pGain = 10U;
    PidB.iGain = 1U;

    PidC.iMax = 2000U;
    PidC.iMin = 0U;
    PidC.pGain = 10U;
    PidC.iGain = 1U;

    Pidspeed.iMax = 300U;
    Pidspeed.iMin = 0U;
    Pidspeed.pGain = 100U; // to be calculated yet
    Pidspeed.iGain = 2U; // to be calculated yet


    // Convert, wait for completion, and store results
    // All the ADC are interrupt initiated SOC. So all the ADC should be software enforced atleast once.
    ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER0);   // Vdd
    ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER1);   // C_i_L1
    ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER2);   // C_i_L2
    ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER3);   // A_v_L1
    ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER4);   // A_v_L2
    ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER5);   // ADC_Imag

    ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER0);   // B_i_L1
    ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER1);   // B_i_L2
    ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER2);   // C_v_L1
    ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER3);   // C_v_L2

    ADC_forceSOC(ADCB_BASE, ADC_SOC_NUMBER0);   // A_i_L1
    ADC_forceSOC(ADCB_BASE, ADC_SOC_NUMBER1);   // A_i_L2
    ADC_forceSOC(ADCB_BASE, ADC_SOC_NUMBER2);   // B_v_L1
    ADC_forceSOC(ADCB_BASE, ADC_SOC_NUMBER3);   // B_v_L2

    //
    // calculations useful for math
    //
    R2 = pow(R, 2);
    lambda_m2 = pow(lambda_m*10, 2)/100;
    L2 = pow(L*100, 2)/10000;

    // field weakening
    gamma = 0;
    //
    // Loop indefinitely
    //
    while(1)
    {
        //GPIO_togglePin(95);

        Tref = (ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5))/2.0 ; //Tref
        if (Tref > 1500)
        {
            Tref = 1500;
        }
        Imag = Tref;

        enc_pos = EQEP_getPosition(EQEP1_BASE);
        theta = enc_pos * 2 * PI * P / max_pos + phase_offset;
        dacVal = 2048 + Imag * sin(theta);
        i_aR = Imag * sin(theta + gamma);
        i_bR = Imag * sin(theta - 2 * PI / 3 + gamma) ;
        i_cR = Imag * sin(theta + 2 * PI / 3 + gamma) ;
        //SPI_writeDataNonBlocking(SPIB_BASE, sdata);
        //DEVICE_DELAY_US(2);

        //
        // store results
        //
        adcResult_Vdd = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0); // Vdd

        adcResult_A_i_L1 = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0); // A_i_L1
        adcResult_A_i_L2 = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1); // A_i_L2

        adcResult_B_i_L1 = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0); // B_i_L1
        adcResult_B_i_L2 = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1); // B_i_L2

        adcResult_C_i_L1 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1); // C_i_L1
        adcResult_C_i_L2 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2); // C_i_L2

        adcResult_A_v_L1 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER3); // A_v_L1
        adcResult_A_v_L2 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER4); // A_v_L2

        adcResult_B_v_L1 = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2); // B_v_L1
        adcResult_B_v_L2 = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER3); // B_v_L2

        adcResult_C_v_L1 = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2); // C_v_L1
        adcResult_C_v_L2 = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER3); // C_v_L2


        i_a = (adcResult_A_i_L1 - adcResult_A_i_L2); // 1Ampere is represented as 100 units
        i_b = (adcResult_B_i_L1 - adcResult_B_i_L2);
        i_c = (adcResult_C_i_L1 - adcResult_C_i_L2);

        errorA = -(i_aR - i_a) ;
        errorB = -(i_bR - i_b) ;
        errorC = -(i_cR - i_c) ;

        v_a = (adcResult_A_v_L1 - adcResult_A_v_L2); // 1 Volt is represented as 100 units
        v_b = (adcResult_B_v_L1 - adcResult_B_v_L2);
        v_c = (adcResult_C_v_L1 - adcResult_C_v_L2);



        speedrpm = posSpeed.speedRPMFR/2.0 ;
        //dacVal = 2*speedrpm;
        DAC_setShadowValue(DACA_BASE, dacVal);

        omega = 5.0*2*PI*speedrpm/60.0 ;
        omega2 = pow(omega,2) ;
        Imag2 = pow(Imag/100,2);



    }

}
// End of Main Loop


//
// Function to configure eQEP1.
//
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
    // Configure the position counter to reset on an maximum position
    //
    EQEP_setPositionCounterConfig(EQEP1_BASE, EQEP_POSITION_RESET_MAX_POS,
                                  max_pos);

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
// End of EQEP


// DAC
void configureDAC(void)
{
    //
    // Set VDAC as the DAC reference voltage.
    // Edit here to use ADC VREF as the reference voltage.
    //
    DAC_setReferenceVoltage(DACA_BASE, DAC_REF_ADC_VREFHI);

    //
    // Enable the DAC output
    //
    DAC_enableOutput(DACA_BASE);

    //
    // Set the DAC shadow output to 0
    //
    DAC_setShadowValue(DACA_BASE, 0);

    //
    // Delay for buffered DAC to power up
    //
    DEVICE_DELAY_US(10);
}
// End of configure for DAC


// Initialize ADCs
void initADCs(void)
{
    //
    // Set ADCCLK divider to /4
    //
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_4_0);

    //
    // Set resolution and signal mode (see #defines above) and load corresponding trims.
    //
    ADC_setMode(ADCA_BASE, EX_ADC_RESOLUTION, EX_ADC_SIGNAL_MODE);
    ADC_setMode(ADCB_BASE, EX_ADC_RESOLUTION, EX_ADC_SIGNAL_MODE);
    ADC_setMode(ADCC_BASE, EX_ADC_RESOLUTION, EX_ADC_SIGNAL_MODE);

    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Powerup the ADCs and then delay for 1 ms
    //
    ADC_enableConverter(ADCA_BASE);
    ADC_enableConverter(ADCB_BASE);
    ADC_enableConverter(ADCC_BASE);

    DEVICE_DELAY_US(1000);

}


void initADCSOCs(void)
{
    //
    // configure SOCs of ADCA
    // - SOC0 will convert pin A0.
    // - SOC1 will convert pin A1.
    // - Both will be triggered by software only.
    // - For 12-bit resolution, a sampling window of 15 ( 75 ns at a 200MHz SYSCLK rate) will be used.
    // - For 16-bit resolution, a sampling window of 64 (320 ns at a 200MHz SYSCLK rate) will be used.
    //

#if(EX_ADC_RESOLUTION == ADC_RESOLUTION_12BIT)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN3, 60);  // Vdd
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN2, 60);  // C_i_L1
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN4, 60);  // C_i_L2
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN14, 60);  // A_v_L1
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN15, 60);  // A_v_L2
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN1, 60);  // Vdd

    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN2, 60);  // A_i_L1
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN4, 60);  // A_i_L2
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN3, 60);  // B_v_L1
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN5, 60);  // B_v_L2

    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN2, 60);  // B_i_L1
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN4, 60);  // B_i_L2
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN3, 60);  // C_v_L1
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN5, 60);  // C_v_L2


#elif(EX_ADC_RESOLUTION == ADC_RESOLUTION_16BIT)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN3,64);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN14, 64); // A_v
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN3, 64);// B_v

    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN2, 64);// A_i
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN2, 64);// B_i
#endif

    //
    // Set Interrupt1 to trigger ADC SOC in all channels
    //
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_ADCINT1);  // Vdd
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_ADCINT1);  // A_v
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_ADCINT1);  // A_v
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_ADCINT1);  // A_v
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER4, ADC_INT_SOC_TRIGGER_ADCINT1);  // A_v
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_ADCINT1);  // A_v

    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_ADCINT1);  // B_v
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_ADCINT1);  // A_i
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_ADCINT1);  // A_i
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_ADCINT1);  // A_i

    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_ADCINT1);  // B_i
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_ADCINT1);  // B_i
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_ADCINT1);  // B_i
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_ADCINT1);  // B_i


    //
    // set SOC1 to set the interrupt 1 flag in A Base.
    // set SOC0 to set the interrupt 1 flag in B Base.
    // set SOC1 to set the interrupt 1 flag in C Base.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER5);
    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER3);
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER3);

    //
    //  Enable the interrupt.
    //
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER1);

    //
    // Set continuous modes for interrupts
    //

    ADC_enableContinuousMode(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_enableContinuousMode(ADCB_BASE, ADC_INT_NUMBER1);
    ADC_enableContinuousMode(ADCC_BASE, ADC_INT_NUMBER1);

    //
    // make sure its flag is cleared.
    //
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

}

//End of functions for ADC

//
// initEPWM1 - Configure ePWM1
//
void initEPWM1()
{

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM1_BASE, EPWM1_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);

    // code for SYNC
    EPWM_setSyncOutPulseMode(EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);


    /*
    // Disable SOCA
    //
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    //
    // Configure the SOC to occur on the first up-count event
    //
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_PERIOD);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);

    //
    // Enable SOCA
    //
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);*/

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, EPWM1_CMP);
    //EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_B, EPWM1_CMP_ADC_TRIG);

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM1_BASE);
    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set-up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    /*EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE, EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);*/

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    /*EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);*/

    // code for Dead band
    EPWM_setDeadBandCounterClock(EPWM1_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
    EPWM_setRisingEdgeDelayCount(EPWM1_BASE, DEAD_BAND);
    EPWM_setFallingEdgeDelayCount(EPWM1_BASE, DEAD_BAND);
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);


    //
    // Intrrupt where we will change the compare values
    // select INT on TIme base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM1_BASE);
    EPWM_setInterruptEventCount(EPWM1_BASE, 1U);

}



//
// initEPWM2 - Configure ePWM2
//
void initEPWM2()
{
    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM2_BASE, EPWM2_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM2_BASE, SHIFT_PHASE);
    EPWM_setTimeBaseCounter(EPWM2_BASE, 0U);

    // code for SYNC
    EPWM_setCountModeAfterSync(EPWM2_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC) ;
    EPWM_setSyncOutPulseMode(EPWM2_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, EPWM2_CMP);
    //EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_B, EPWM2_CMP);

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM2_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM2_BASE);
    EPWM_setClockPrescaler(EPWM2_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set-up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    /*EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE, EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);*/
    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    /*EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);*/

    // code for Dead band
    EPWM_setDeadBandCounterClock(EPWM2_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
    EPWM_setRisingEdgeDelayCount(EPWM2_BASE, DEAD_BAND);
    EPWM_setFallingEdgeDelayCount(EPWM2_BASE, DEAD_BAND);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_RED,
                                  EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_FED,
                                  EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM2_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM2_BASE, EPWM_DB_INPUT_EPWMA);


    //
    // Interrupt where we will change the compare values
    // select INT on TIme base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    EPWM_setInterruptSource(EPWM2_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM2_BASE);
    EPWM_setInterruptEventCount(EPWM2_BASE, 1U);
}



//
// initEPWM3 - Configure ePWM2
//
void initEPWM3()
{
    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM3_BASE, EPWM3_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM3_BASE, SHIFT_PHASE);
    EPWM_setTimeBaseCounter(EPWM3_BASE, 0U);

    // code for SYNC
    EPWM_setCountModeAfterSync(EPWM3_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC) ;
    EPWM_setSyncOutPulseMode(EPWM3_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, EPWM3_CMP);
    //EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_B, EPWM2_CMP);

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM3_BASE);
    EPWM_setClockPrescaler(EPWM3_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set-up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    /*EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE, EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);*/
    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    /*EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);*/

    // code for Dead band
    EPWM_setDeadBandCounterClock(EPWM3_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
    EPWM_setRisingEdgeDelayCount(EPWM3_BASE, DEAD_BAND);
    EPWM_setFallingEdgeDelayCount(EPWM3_BASE, DEAD_BAND);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_RED,
                                  EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_FED,
                                  EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM3_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM3_BASE, EPWM_DB_INPUT_EPWMA);

    //
    // Intrrupt where we will change the compare values
    // select INT on TIme base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    EPWM_setInterruptSource(EPWM3_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM3_BASE);
    EPWM_setInterruptEventCount(EPWM3_BASE, 1U);
}



//
// initEPWM4 - Configure ePWM4
//
void initEPWM4()
{
    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM4_BASE, EPWM4_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM4_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM4_BASE, 0U);

    // code for SYNC
    EPWM_setCountModeAfterSync(EPWM4_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC) ;
    SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
    EPWM_setSyncOutPulseMode(EPWM4_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, EPWM4_CMP);
    //EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_B, EPWM2_CMP);

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM4_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM4_BASE);
    EPWM_setClockPrescaler(EPWM4_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set-up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM4_BASE, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    /*EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE, EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);*/
    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    /*EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);*/

    // code for Dead band
    EPWM_setDeadBandCounterClock(EPWM4_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
    EPWM_setRisingEdgeDelayCount(EPWM4_BASE, DEAD_BAND);
    EPWM_setFallingEdgeDelayCount(EPWM4_BASE, DEAD_BAND);
    EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayPolarity(EPWM4_BASE, EPWM_DB_RED,
                                  EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM4_BASE, EPWM_DB_FED,
                                  EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM4_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM4_BASE, EPWM_DB_INPUT_EPWMA);


    //
    // Intrrupt where we will change the compare values
    // select INT on TIme base counter zero event,
    // Enable INT, generate INT on 3rd event
    /*
    EPWM_setInterruptSource(EPWM4_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM4_BASE);
    EPWM_setInterruptEventCount(EPWM4_BASE, 1U);*/
}

//
// initEPWM5 - Configure ePWM4
//
void initEPWM5()
{
    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM5_BASE, EPWM5_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM5_BASE, SHIFT_PHASE);
    EPWM_setTimeBaseCounter(EPWM5_BASE, 0U);

    // code for SYNC
    EPWM_setCountModeAfterSync(EPWM5_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC) ;
    //SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM5, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
    EPWM_setSyncOutPulseMode(EPWM5_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A, EPWM5_CMP);
    //EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_B, EPWM2_CMP);

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM5_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM5_BASE);
    EPWM_setClockPrescaler(EPWM5_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set-up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM5_BASE, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    /*EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE, EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);*/
    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    /*EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);*/

    // code for Dead band
    EPWM_setDeadBandCounterClock(EPWM5_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
    EPWM_setRisingEdgeDelayCount(EPWM5_BASE, DEAD_BAND);
    EPWM_setFallingEdgeDelayCount(EPWM5_BASE, DEAD_BAND);
    EPWM_setDeadBandDelayMode(EPWM5_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM5_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayPolarity(EPWM5_BASE, EPWM_DB_RED,
                                  EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM5_BASE, EPWM_DB_FED,
                                  EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM5_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM5_BASE, EPWM_DB_INPUT_EPWMA);


    //
    // Intrrupt where we will change the compare values
    // select INT on TIme base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    /*EPWM_setInterruptSource(EPWM5_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM5_BASE);
    EPWM_setInterruptEventCount(EPWM5_BASE, 1U);*/

}



//
// initEPWM6 - Configure ePWM4
//
void initEPWM6()
{
    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM6_BASE, EPWM5_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM6_BASE, SHIFT_PHASE);
    EPWM_setTimeBaseCounter(EPWM6_BASE, 0U);

    // code for SYNC
    EPWM_setCountModeAfterSync(EPWM6_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC) ;
    //SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM5, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
    EPWM_setSyncOutPulseMode(EPWM6_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, EPWM6_CMP);
    //EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_B, EPWM2_CMP);

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM6_BASE);
    EPWM_setClockPrescaler(EPWM6_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set-up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM6_BASE, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    /*EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE, EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);*/
    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    /*EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);*/

    // code for Dead band
    EPWM_setDeadBandCounterClock(EPWM6_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
    EPWM_setRisingEdgeDelayCount(EPWM6_BASE, DEAD_BAND);
    EPWM_setFallingEdgeDelayCount(EPWM6_BASE, DEAD_BAND);
    EPWM_setDeadBandDelayMode(EPWM6_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM6_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayPolarity(EPWM6_BASE, EPWM_DB_RED,
                                  EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM6_BASE, EPWM_DB_FED,
                                  EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM6_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM6_BASE, EPWM_DB_INPUT_EPWMA);


    //
    // Intrrupt where we will change the compare values
    // select INT on TIme base counter zero event,
    // Enable INT, generate INT on 3rd event
    /*
    EPWM_setInterruptSource(EPWM6_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM6_BASE);
    EPWM_setInterruptEventCount(EPWM6_BASE, 1U);*/

}




//
// epwm1ISR-- ePWM 1 ISR
//
__interrupt void epwm1ISR(void)
{
    //uint16_t i;

    //
    // Position speed and measurement
    //
    PosSpeed_calculate(&posSpeed);

    CMPA = UpdatePID(&PidA, errorA, i_aR);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, CMPA);
    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, CMPA);
    //EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, CMP);



//    enc_pos = EQEP_getPosition(EQEP1_BASE);
//    dacVal = 2048 + 2000 * sin(enc_pos * 2 * PI * 5.0 / max_pos);

/*    if(i_a > (i_aR + Iband))
    {
        EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 2000U);
        EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, 2000U);
        loopcount = 1;
    }

    if(i_a < (i_aR - Iband))
    {
        EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, 0U);
        EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0U);
        loopcount = 2;
    }
*/
    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    //
    // Acknowledge interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

//
// epwm2ISR-- ePWM 2 ISR
//
__interrupt void epwm2ISR(void)
{
    CMPB = UpdatePID(&PidB, errorB, i_bR);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, CMPB);
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A, CMPB);
    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM2_BASE);

    //
    // Acknowledge interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}


//
// epwm2ISR-- ePWM 3 ISR
//
__interrupt void epwm3ISR(void)
{
    CMPC = UpdatePID(&PidC, errorC, i_cR);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, CMPC);
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, CMPC);
    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM3_BASE);

    //
    // Acknowledge interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}


//
// Call this function to update the PI values
//

double UpdatePID(SPid * pid, double error, double current)
{
    double pTerm,  iTerm, Duty; // dTerm,
    pTerm = pid->pGain * error; // calculate the proportional term
    // calculate the integral state with appropriate limiting
    pid->iState += error;
    if (pid->iState > pid->iMax)
        pid->iState = pid->iMax;
    else if (pid->iState < pid->iMin)
        pid->iState = pid->iMin;
    iTerm = pid->iGain * pid->iState; // calculate the integral term
    Duty = pTerm + iTerm;
    //dTerm = pid->dGain * (pid->dState - position);
    //pid->dState = position;
    if (Duty > pid->iMax)
        Duty = pid->iMax;
    else if (Duty < pid->iMin)
        Duty = pid->iMin;
    return Duty;
}


//
// End of file
//
