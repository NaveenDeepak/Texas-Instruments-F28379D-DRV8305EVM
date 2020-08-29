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

// these are for generating PWM for eQEP
#define TB_CLK  DEVICE_SYSCLK_FREQ / 2              // Time base clock is SYCLK / 2
#define PWM_CLK 5000                                // We want to output at 5 kHz
                                                    // (300 rpm)
#define PRD_VAL (TB_CLK / (PWM_CLK * 2))            // Calculate period value for up-down count mode

// .9999 / 4000 converted to IQ26 fixed point format
#define MECH_SCALER     16776
// 2 pole pairs in this example
#define POLE_PAIRS      2
// Angular offset between encoder and Phase A
#define CAL_ANGLE       0
// See Equation 5 in eqep_ex2_calculation.c
#define SPEED_SCALER    ((((uint64_t)32 * DEVICE_SYSCLK_FREQ / 64) * 60) / (24000000))
// Base/max rpm is 6000rpm
#define BASE_RPM        6000



// these are for lookup tables
#define CPUFREQ_MHZ           200
#define PI                    3.14159265
#define SINE_TBL_SIZE         360

//
// GLOBALS
//
uint16_t interruptCount = 0;
uint16_t loopcount = 0;
uint16_t CMP = 1000U;

// these are for lookup tables
uint16_t SINE_TBL[SINE_TBL_SIZE];
uint16_t tableStep = 1;
float waveformGain = 0.8003; // Range 0.0 -> 1.0
float waveformOffset = 0;    // Range -1.0 -> 1.0

// these are for eQEP
uint16_t enc_pos;
uint16_t max_pos = 3999;
// these are for reference currents
float i_a;
float i_b;
float i_c;

// SPI
uint16_t sData[2] ;
uint16_t rData[2] ;
uint16_t rDataPoint = 0;

// these are for DAC
uint16_t dacVal = 2048;


//
//Function Prototypes
//

// these are for EPWM
//void initEPWM(void);
//__interrupt void epwmISR(void);

// these are for lookup tables
void configureWaveform(void);

// these are for eQEP
void initEQEP(void);

// these are for inverter communication
void initSPIBMaster(void);
void initSPIASlave(void);
void configGPIOs(void);
__interrupt void spibTxFIFOISR(void);
__interrupt void spiaRxFIFOISR(void);

// these are for DAC
void configureDAC(void);



//
// Main
//
void main(void)
{

    uint16_t i;
    //
    // Initiate device clock and periperals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();

    /*
    // Initialize GPIO0 to ePWM1A, GPIO1 to ePWM1B, and GPIO4 as an output.
    // They will be used to simulate incoming eQEP Phase A, Phase B, and Index
    // signals respectively.
    //
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    GPIO_setPinConfig(GPIO_1_EPWM1B);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);*/



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
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    //Initialize the PIE vector table with pointers to the shell interrupt Interrupt Service Routines (ISR)
    //
    Interrupt_initVectorTable();

    //
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file
    //
    //Interrupt_register(INT_EPWM1, &epwmISR);
    Interrupt_register(INT_SPIB_TX, &spibTxFIFOISR);
    Interrupt_register(INT_SPIA_RX, &spiaRxFIFOISR);

    //
    // Setup SPI, initializing it for FIFO mode
    //
    configGPIOs();
    initSPIBMaster();
    initSPIASlave();

    //
    // Initialize the data buffers
    //
    for (i = 0; i < 2; i++)
    {
        sData[i] = i;
        rData[i] = 0;
    }

    //
    // Setup ePWM1 to generate a 5 kHz signal to be an input to the eQEP
    //
    //initEPWM();

    //
    // Setup eQEP1, configuring the unit timer and quadrature capture units
    //
    initEQEP();

    //
    // Configure the DAC module
    //
    configureDAC();


    //
    // Enable ePWM interrupts
    //
    //Interrupt_enable(INT_EPWM1);
    Interrupt_enable(INT_SPIB_TX);
    Interrupt_enable(INT_SPIA_RX);

    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT ;
    ERTM ;

    configureWaveform();

    //
    // Loop indefinitely
    //
    while(1)
    {
        enc_pos = EQEP_getPosition(EQEP1_BASE);
        dacVal = 2048 + 2000 * sin(enc_pos * 2 * PI * 5.0 / max_pos);
        DAC_setShadowValue(DACA_BASE, dacVal);
        i_a = sin(enc_pos * PI / 2000.0);
        i_b = sin(enc_pos * PI / 2000.0 - 2 * PI / 3);
        i_c = sin(enc_pos * PI / 2000.0 + 2 * PI / 3);
        DEVICE_DELAY_US(2);

        /*if(loopcount > 8000)
        {
            //ESTOP0;
            CMP = EPWM1_CMP + 500U;
            EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, CMP);
            loopcount = 0;
        }

        loopcount = loopcount + 1;*/

    }

}



// initEPWM1 - Configure ePWM1
//
void initEPWM(void)
{

    //
    // Disable the ePWM time base clock before configuring the module
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Set phase shift to 0 and clear the time base counter
    //
    EPWM_setPhaseShift(EPWM1_BASE, 0);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0);

    //
    // Disable the shadow load; the load will be immediate instead
    //
    EPWM_disableCounterCompareShadowLoadMode(EPWM1_BASE,
                                             EPWM_COUNTER_COMPARE_A);
    EPWM_disableCounterCompareShadowLoadMode(EPWM1_BASE,
                                             EPWM_COUNTER_COMPARE_B);

    //
    // Set the compare A value to half the period value, compare B to 0
    //
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A,
                                PRD_VAL / 2);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_B, 0);

    //
    // Set action qualifier behavior on compare A events
    // - EPWM1A --> 1 when CTR = CMPA and increasing
    // - EPWM1A --> 0 when CTR = CMPA and decreasing
    //
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    //
    // Set action qualifier behavior on compare B events
    // - EPWM1B --> 1 when CTR = PRD and increasing
    // - EPWM1B --> 0 when CTR = 0 and decreasing
    //
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    //
    // Enable interrupt when the counter is equal to PRD
    //
    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_PERIOD);
    EPWM_enableInterrupt(EPWM1_BASE);

    //
    // Interrupt on first event
    //
    EPWM_setInterruptEventCount(EPWM1_BASE, 1);

    //
    // Set the time base clock prescaler to /1
    //
    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set the period value; don't shadow the register
    //
    EPWM_setPeriodLoadMode(EPWM1_BASE, EPWM_PERIOD_DIRECT_LOAD);
    EPWM_setTimeBasePeriod(EPWM1_BASE, PRD_VAL);

    //
    // Put the time base counter into up-down count mode
    //
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    //
    // Sync the ePWM time base clock
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

}



//
// epwm1ISR-- ePWM 1 ISR
//
__interrupt void epwmISR(void)
{



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
// configureWaveform - Configure the SINE waveform
//
void configureWaveform(void)
{
    uint16_t j;

    //
    // Fill Sine Table
    //
    for(j=0;j<SINE_TBL_SIZE;j++)
    {
        SINE_TBL[j] = (sin(j*PI/180.0)+1.0)*2047.5;
    }

    /*
    // Adjust for Gain and Offset
    //
    offset = (SINE_TBL[0] - (SINE_TBL[0]*waveformGain)) + (SINE_TBL[0]*waveformOffset);

    for(j=0;j<SINE_TBL_SIZE;j++)
    {
        waveformValue = (SINE_TBL[j]*waveformGain)+offset;
        SINE_TBL[j] = waveformValue < 0 ? 0 : waveformValue > 4095 ? 4095 : waveformValue;
    }*/
}

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
    // Configure the position counter to reset on an index event
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



// Function to configure SPI B in FIFO mode MASTER.
//
void initSPIBMaster()
{
    //
    // Must put SPI into reset before configuring it
    //
    SPI_disableModule(SPIB_BASE);

    //
    // SPI configuration. Use a 1MHz SPICLK and 16-bit word size.
    //
    SPI_setConfig(SPIB_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_MASTER, 500000, 16);
    SPI_enableLoopback(SPIB_BASE);
    SPI_setEmulationMode(SPIB_BASE, SPI_EMULATION_FREE_RUN);

    //
    // FIFO and interrupt configuration
    //
    SPI_enableFIFO(SPIB_BASE);
    SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_TXFF );
    SPI_setFIFOInterruptLevel(SPIB_BASE, SPI_FIFO_TX2, SPI_FIFO_RX2);
    SPI_enableInterrupt(SPIB_BASE, SPI_INT_TXFF);

    //
    // configuration complete, Enable the module.
    //
    SPI_enableModule(SPIB_BASE);

}

//
// Function to configure SPI A in FIFO mode.
//
void initSPIASlave()
{
    //
    // Must put SPI into reset before configuring it
    //
    SPI_disableModule(SPIA_BASE);

    //
    // SPI configuration. Use a 1MHz SPICLK and 16-bit word size.
    //
    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_SLAVE, 500000, 16);
    SPI_enableLoopback(SPIA_BASE);
    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_FREE_RUN);

    //
    // FIFO and interrupt configuration
    //
    SPI_enableFIFO(SPIA_BASE);
    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RXFF );
    SPI_setFIFOInterruptLevel(SPIA_BASE, SPI_FIFO_TX2, SPI_FIFO_RX2);
    SPI_enableInterrupt(SPIA_BASE, SPI_INT_RXFF);

    //
    // configuration complete, Enable the module.
    //
    SPI_enableModule(SPIA_BASE);

}

//
// Configure GPIOs for external loopback.
//
void configGPIOs(void)
{
    //
    // This test is designed for an external loopback between SPIA
    // and SPIB.
    // External Connections:
    // -GPIO25 and GPIO17 - SPISOMI
    // -GPIO24 and GPIO16 - SPISIMO
    // -GPIO23 and GPIO19 - SPISTE
    // -GPIO22 and GPIO18 - SPICLK
    //

    //
    // GPIO59 is the SPISOMIA.
    //
    GPIO_setMasterCore(59, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_59_SPISOMIA);
    GPIO_setPadConfig(59, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(59, GPIO_QUAL_ASYNC);

    //
    // GPIO58 is the SPISIMOA clock pin.
    //
    GPIO_setMasterCore(58, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_58_SPISIMOA);
    GPIO_setPadConfig(58, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(58, GPIO_QUAL_ASYNC);

    //
    // GPIO61 is the SPISTEA.
    //
    GPIO_setMasterCore(61, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_61_SPISTEA);
    GPIO_setPadConfig(61, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(61, GPIO_QUAL_ASYNC);

    //
    // GPIO60 is the SPICLKA.
    //
    GPIO_setMasterCore(60, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_60_SPICLKA);
    GPIO_setPadConfig(60, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(60, GPIO_QUAL_ASYNC);

    //
    // GPIO64 is the SPISOMIB.
    //
    GPIO_setMasterCore(64, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_64_SPISOMIB);
    GPIO_setPadConfig(64, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(64, GPIO_QUAL_ASYNC);

    //
    // GPIO63 is the SPISIMOB clock pin.
    //
    GPIO_setMasterCore(63, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_63_SPISIMOB);
    GPIO_setPadConfig(63, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(63, GPIO_QUAL_ASYNC);

    //
    // GPIO66 is the SPISTEB.
    //
    GPIO_setMasterCore(66, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_66_SPISTEB);
    GPIO_setPadConfig(66, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(66, GPIO_QUAL_ASYNC);

    //
    // GPIO65 is the SPICLKB.
    //
    GPIO_setMasterCore(65, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_65_SPICLKB);
    GPIO_setPadConfig(65, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(65, GPIO_QUAL_ASYNC);
}

//
// SPI B transmit FIFO ISR
//
__interrupt void spibTxFIFOISR(void)
{
    uint16_t i;
    //
    // Send data
    //
    for(i = 0; i<2; i++)
    {
        SPI_writeDataNonBlocking(SPIB_BASE, sData[i]);
    }

    //
    // Increment data for next cycle
    //
    for (i = 0; i < 2; i++)
    {
        sData[i] = sData[i] + 1;
    }

    //
    // Clear Interrupt flag and issue ACK
    //
    SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);

}

//
// SPI A RECEIVE FIFO ISR
//

__interrupt void spiaRxFIFOISR(void)
{
    uint16_t i;

    //
    // Read data
    //
    for(i=0; i <2; i++)
    {
        rData[i] = SPI_readDataNonBlocking(SPIA_BASE);
    }

    //
    // Check received data
    //
    for (i = 0; i < 2; i++)
    {
        if (rData[i] != (rDataPoint + i))
        {
            // Something went wrong. rData doesn't contain expected data.
            ESTOP0;
        }
    }

    rDataPoint++;

    //
    // Clear interrupt flag and issue ACK
    //
    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);
}


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

//
// End of file
//
