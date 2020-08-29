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

#define EPWM4_TIMER_TBPRD   2000U
#define EPWM4_CMP           1000U

#define DEAD_BAND           50U


// these are for lookup tables
#define CPUFREQ_MHZ           200
#define PI                    3.14159265

//
// GLOBALS
//
float adcResult_Vdd;
float adcResult_A_v_L1;
float adcResult_A_v_L2;
float adcResult_A_i_L1;
float adcResult_A_i_L2;
float adcResult_B_v;
float adcResult_B_i;

uint16_t interruptCount = 0;
uint16_t loopcount = 0;
uint16_t Mainloopcount = 0;
uint16_t EPWM1loopcount = 0;
uint16_t EPWM2loopcount = 0;
uint16_t CMP = 1000U;

// these are for eQEP
uint16_t enc_pos;
uint16_t max_pos = 3999;

// these are for reference currents
float i_aR;
float i_bR;
float i_cR;
float Imag = 100;
float Iband = 10;
float phase_offset = (1.0 - 0.15)*PI;  // to rectify encoder offset with respect to phase A

// these are for measured currents
float i_a = 0;
float i_b = 0;
float i_c = 0;

// these are for DAC
float dacVal = 2048;

//
// PID variables
//

typedef struct
{
    double dState;  // Last position input
    double iState;  // Integrator state
    double iMax, iMin;  // Maximum and minimum allowable integrator state

    double iGain,   // integral gain
           pGain,   // proportional gain
           dGain;   // derivative gain
} SPid;

float errorA = 0;

SPid PidA;
SPid PidB;
SPid PidC;

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
void initEPWM4(void);
void initEPWM5(void);
void initEPWM6(void);

__interrupt void epwm1ISR(void);
__interrupt void epwm2ISR(void);
__interrupt void epwm4ISR(void);


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
    // Configure GPIO0/1, and GPIO2/3 as ePWM1A/1B, ePWM2A/2B
    //
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_1_EPWM1B);

    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_2_EPWM2A);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_3_EPWM2B);

    //
    // Configure GPIO6/7 as ePWM4A/B to emulate encoder signals
    //
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_6_EPWM4A);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_7_EPWM4B);

    //
    // Configure GPIO8/9 as ePWM5A/B to emulate encoder signals
    //
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_8_EPWM5A);
    //GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);
    //GPIO_setPinConfig(GPIO_9_EPWM5B);

    //
    // Configure GPIO10/11 as ePWM6A/B to emulate encoder signals
    //
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_10_EPWM6A);
    //GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);
    //GPIO_setPinConfig(GPIO_11_EPWM6B);

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
    Interrupt_register(INT_EPWM4, &epwm4ISR);


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
    Interrupt_enable(INT_EPWM4);

    /*Interrupt_enable(INT_ADCA1);
    Interrupt_enable(INT_ADCB1);
    Interrupt_enable(INT_ADCC1);*/

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
    PidA.pGain = 2U;
    PidA.iGain = 1U;


    // Convert, wait for completion, and store results
    // All the ADC are interrupt initiated SOC. So all the ADC should be software enforced atleast once.
    ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER0);   // Vdd
    ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER1);   // A_v_L1
    ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER2);   // A_v_L2
    ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER0);   // B_v
    ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER1);   // A_i_L1
    ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER2);   // A_i_L2
    ADC_forceSOC(ADCB_BASE, ADC_SOC_NUMBER0);   // B_i

    //GPIO_setPadConfig(95, GPIO_PIN_TYPE_STD);
    //GPIO_setDirectionMode(95, GPIO_DIR_MODE_OUT);

    //
    // Loop indefinitely
    //
    while(1)
    {
        //GPIO_togglePin(95);

        enc_pos = EQEP_getPosition(EQEP1_BASE);
        dacVal = 2048 + Imag * sin(enc_pos * 2 * PI * 5.0 / max_pos + phase_offset);
        i_aR = Imag * sin(enc_pos * 2 * PI * 5.0 / max_pos + phase_offset );
        i_bR = Imag * sin(enc_pos * 2 * PI * 5.0 / max_pos - 2 * PI / 3 + phase_offset);
        i_cR = Imag * sin(enc_pos * 2 * PI * 5.0 / max_pos + 2 * PI / 3 + phase_offset);
        //SPI_writeDataNonBlocking(SPIB_BASE, sdata);
        //DEVICE_DELAY_US(2);

        //
        // store results
        //

        adcResult_Vdd = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0); // Vdd
        adcResult_A_v_L1 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1); // A_v_L1
        adcResult_A_v_L2 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2); // A_v_L2
        adcResult_B_v = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0); // B_v
        adcResult_A_i_L1 = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1); // A_i_L1
        adcResult_A_i_L2 = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2); // A_i_L2
        adcResult_B_i = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0); // B_i

        i_a = (adcResult_A_i_L1 - adcResult_A_i_L2); // 1Ampere is represented as 100 units
        errorA = -(i_aR - i_a) ;
        DAC_setShadowValue(DACA_BASE, dacVal);

        /*if(loopcount > 8000)
        {
            //ESTOP0;
        }*/

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
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN14, 60);  // A_v_L1
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN15, 60);  // A_v_L2

    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN3, 60);  // B_v
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN2, 60);  // A_i_L1
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN4, 60);  // A_i_L2

    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN2, 60);  // B_i


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
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_ADCINT1);  // B_v
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_ADCINT1);  // A_i
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_ADCINT1);  // A_i
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_ADCINT1);  // B_i

    //
    // set SOC1 to set the interrupt 1 flag in A Base.
    // set SOC0 to set the interrupt 1 flag in B Base.
    // set SOC1 to set the interrupt 1 flag in C Base.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2);
    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2);

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
    EPWM_setPhaseShift(EPWM2_BASE, 0U);
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
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE,
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
    // Intrrupt where we will change the compare values
    // select INT on TIme base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    EPWM_setInterruptSource(EPWM2_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM2_BASE);
    EPWM_setInterruptEventCount(EPWM2_BASE, 1U);
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
    //
    EPWM_setInterruptSource(EPWM4_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM4_BASE);
    EPWM_setInterruptEventCount(EPWM4_BASE, 1U);

}


//
// initEPWM5 - Configure ePWM5 to emulate Encoder signals
//

void initEPWM5(void)
{

    //
    // Set phase shift to 0 and clear the time base counter
    //
    EPWM_setPhaseShift(EPWM5_BASE, 0);
    EPWM_setTimeBaseCounter(EPWM5_BASE, 0);

    // code for SYNC
    EPWM_setCountModeAfterSync(EPWM5_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC) ;
    EPWM_setSyncOutPulseMode(EPWM5_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    //
    // Disable the shadow load; the load will be immediate instead
    //
    EPWM_disableCounterCompareShadowLoadMode(EPWM5_BASE,
                                             EPWM_COUNTER_COMPARE_A);
    //EPWM_disableCounterCompareShadowLoadMode(EPWM5_BASE,
    //                                         EPWM_COUNTER_COMPARE_B);

    //
    // Set the compare A value to half the period value, compare B to 0
    //
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A,
                                PRD_VAL / 2);
    //EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_B, 0);

    //
    // Set action qualifier behavior on compare A events
    // - EPWM1A --> 1 when CTR = CMPA and increasing
    // - EPWM1A --> 0 when CTR = CMPA and decreasing
    //
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    //
    // Set action qualifier behavior on compare B events
    // - EPWM1B --> 1 when CTR = PRD and increasing
    // - EPWM1B --> 0 when CTR = 0 and decreasing
    //
    /*EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);*/

    //
    // Set the time base clock prescaler to /1
    //
    EPWM_setClockPrescaler(EPWM5_BASE, EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set the period value; don't shadow the register
    //
    EPWM_setPeriodLoadMode(EPWM5_BASE, EPWM_PERIOD_DIRECT_LOAD);
    EPWM_setTimeBasePeriod(EPWM5_BASE, PRD_VAL);
    //
    // Put the time base counter into up-down count mode
    //
    EPWM_setTimeBaseCounterMode(EPWM5_BASE, EPWM_COUNTER_MODE_UP_DOWN);

}


//
// initEPWM6 - Configure ePWM6 to emulate Encoder signals
//

void initEPWM6(void)
{

    //
    // Set phase shift to 0 and clear the time base counter
    //
    EPWM_setTimeBasePeriod(EPWM6_BASE, PRD_VAL);
    EPWM_setPhaseShift(EPWM6_BASE, 0);
    EPWM_setTimeBaseCounter(EPWM6_BASE, 0);

    //
    EPWM_setCountModeAfterSync(EPWM6_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC) ;

    //
    // Set the compare A value to half the period value, compare B to 0
    //
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A,0);
    //EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_B, 0);

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM6_BASE);
    EPWM_setClockPrescaler(EPWM6_BASE, EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set-up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM4_BASE, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    //EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE, EPWM_COUNTER_COMPARE_B,
    // EPWM_COMP_LOAD_ON_CNTR_ZERO);


    //
    // Set action qualifier behavior on compare A events
    // - EPWM1A --> 1 when CTR = CMPA and increasing
    // - EPWM1A --> 0 when CTR = CMPA and decreasing
    //
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    //
    // Set action qualifier behavior on compare B events
    // - EPWM1B --> 1 when CTR = PRD and increasing
    // - EPWM1B --> 0 when CTR = 0 and decreasing
    //
    /*EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);*/

}



//
// epwm1ISR-- ePWM 1 ISR
//
__interrupt void epwm1ISR(void)
{


    CMP = UpdatePID(&PidA, errorA, i_aR);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, CMP);
    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, CMP);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, CMP);



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
// epwm4ISR-- ePWM 4 ISR
//
__interrupt void epwm4ISR(void)
{
    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM4_BASE);

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
