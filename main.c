/*
 * res_2_voltage.c
 * November 17, 2022
 * Version 1.0
 * Author: Jonathan Valdez (jonathanlvaldez@gmail.com)
 *
 * Description: A simple software that sampels the ADC values from the thermistors, and then
 * will generate a PWM output to scale the temperature range to a 0 to 1V signal needed for
 * the Gen 3 LS ECU O2 sensor voltage scale.
 *
 * Pinout:
 *   Inputs:
 *     - P1.3 : Thermistor 1
 *     - P1.4 : Thermistor 2
 *
 *   Outputs:
 *     - P1.6/P2.0 : PWM Out 1
 *     - P1.7/P2.1 : PWM Out 2
 *
 *
 * Software Details:
 *   - ADC is triggered by TB1.1 every 100 ms to start a sample.
 *   - Once an ADC sample is complete, an interrupt fires to tell software to calculate the
 *     PWM duty cycle needed to achieve desired voltage based on a resistance to temperature LUT.
 */

#include <msp430.h>
#include <gpio.h>
#include <timer_b.h>
#include <adc.h>

// Output voltage of GPIOs (for generating analog voltage output)
#define SYS_VCC_VOLTAGE_MV                      3300
#define SYS_ADC_INPUT_PULL_UP_R                 2490
#define SYS_ADC_MAX_VALUE                       1024
#define SYS_PWM_MAX_VALUE                       1024                    // The maximum value that the PWM can generate
#define SYS_PWM_MAX_TARGET_VOLTAGE_MV           1000                    // The output voltage we want for maximum temp value
#define SYS_PWM_MIN_TARGET_VOLTAGE_MV           0                       // The output voltage we want for minimum temp value
#define SYS_PWM_MAX_TARGET_TEMP_F               ((150 * 9 / 5) + 32)    // The temperature we want to represent at the SYS_PWM_MAX_TARGET_VOLTAGE_MV
#define SYS_PWM_MIN_TARGET_TEMP_F               ((0 * 9 / 5) + 32)      // The temperature we want to represent at the SYS_PWM_MIN_TARGET_VOLTAGE_MV

// Each timer period is ~1 ms. We aim to sample every 100 ms
#define NUM_TIMER_OVERFLOWS_BETWEEN_SAMPLES     100

void initClocks(void);
void initGPIOs(void);
void initTimers(void);
void initADC(void);
void timerOverflowChecker(void);

/* Functions for math */
uint32_t calculateResistance(uint16_t adcSample);
int16_t calculateTemperature(uint32_t resistance);
uint16_t calculateTargetOutputVoltageFromTemp(int16_t temp);
uint16_t calculatePWMDutyCycleFromTargetOutputVoltage(uint16_t targetmv);

/* Initialize some global variables used to start the ADC samples and which input is sampled */
volatile uint8_t timerOverflowCount;
volatile uint8_t adcSampleInput1;                               // This is calling out channel 1, so that if it's 0, then we use index 0.

/* Initialize some global variables to store the ADC values */
volatile uint16_t adcSample[2];
volatile uint8_t adcNewSample;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                                       // Stop WDT
    initClocks();                                                   // Initialize the MCU clocks
    initGPIOs();                                                    // Initialize the device GPIOs
    initADC();                                                      // Initialize the ADC
    timerOverflowCount = 0;
    adcSampleInput1 = 0;
    adcSample[0] = 0;
    adcSample[1] = 0;
    adcNewSample = 0;

    initTimers();                                                   // Initialize the PWM timers and ADC timers

    Timer_B_startCounter(TIMER_B0_BASE, TIMER_B_CONTINUOUS_MODE);   // Start the timer

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;


    while (1)
    {
        if (adcNewSample & 0x01)
        {
            /* New sample is here for input 1 for us to do some math and update the PWM output for */
            uint32_t resistance = calculateResistance(adcSample[0]);
            int32_t temperature = calculateTemperature(resistance);
            uint16_t targetmv = calculateTargetOutputVoltageFromTemp(temperature);
            uint16_t pwmCount = calculatePWMDutyCycleFromTargetOutputVoltage(targetmv);
            Timer_B_setCompareValue(TIMER_B0_BASE, TIMER_B_CAPTURECOMPARE_REGISTER_1, pwmCount);

            adcNewSample &= ~0x01;
        }
        if (adcNewSample & 0x02)
        {
            /* New sample is here for input 2 for us to do some math and update the PWM output for */
            uint32_t resistance = calculateResistance(adcSample[1]);
            int32_t temperature = calculateTemperature(resistance);
            uint16_t targetmv = calculateTargetOutputVoltageFromTemp(temperature);
            uint16_t pwmCount = calculatePWMDutyCycleFromTargetOutputVoltage(targetmv);
            Timer_B_setCompareValue(TIMER_B0_BASE, TIMER_B_CAPTURECOMPARE_REGISTER_2, pwmCount);

            adcNewSample &= ~0x02;
        }
    }
}

uint32_t calculateResistance(uint16_t adcSample)
{
    int32_t samplemv = (adcSample * SYS_VCC_VOLTAGE_MV) / SYS_ADC_MAX_VALUE;
    int32_t calculatedResistance = (-1*SYS_ADC_INPUT_PULL_UP_R*samplemv)/(samplemv-SYS_VCC_VOLTAGE_MV);
    return calculatedResistance;
}

int16_t calculateTemperature(uint32_t resistance)
{

    int32_t res = 0;

    if (resistance <= 0x7FFFFFFF)
        res = resistance;


#define numberOfMeasurements        20

    /* Critical: Due to my laziness, this list should always have descending resistance values!!
     * These are expected to be in degrees Celcius. Conversion happens later to gain more resolution */
    const int32_t tempCurve[numberOfMeasurements][2] = {
                                    { -40, 96044 },
                                    { -30, 53674 },
                                    { -20, 26386 },
                                    { -10, 14822 },
                                    {   0, 91247 },
                                    {  10, 5244  },
                                    {  20, 3277  },
                                    {  30, 2108  },
                                    {  40, 1392  },
                                    {  50, 941   },
                                    {  60, 650   },
                                    {  70, 458   },
                                    {  80, 329   },
                                    {  90, 240   },
                                    { 100, 178   },
                                    { 110, 134   },
                                    { 120, 102   },
                                    { 130, 78    },
                                    { 140, 61    },
                                    { 150, 48    }

    };




    /* Now to interpolate to the closest value */
    uint8_t closestLower = 255;

    uint8_t i;

    /* Find the first index that has a value below the given */
    for (i = 0; i < numberOfMeasurements; i++)
    {
        if (tempCurve[i][1] <= res)
        {
            closestLower = i;
            break;
        }
    }

    /* A few checks for invalid corner cases */
    if (closestLower == 255)      // All values were above the measured resistance. Return last temp
        return tempCurve[numberOfMeasurements-1][0];

    if (closestLower == 0)                  // If first one was lower, then we measured higher resistance than we have. Return first
        return tempCurve[0][0];

    if (tempCurve[closestLower][1] == res)  // If an exact match, save the CPU cycles and return the value
        return tempCurve[closestLower][0];



     /* Now we interpolate the found index, and 1 less than the index (should be above it) */
    int32_t y1 = tempCurve[closestLower-1][1];
    int32_t x1 = (tempCurve[closestLower-1][0] * 9 / 5) + 32;
    int32_t y2 = tempCurve[closestLower][1];
    int32_t x2 = (tempCurve[closestLower][0] * 9 / 5) + 32;

    /* Shouldn't ever have the below be true */
    if (y1 <= y2)
        return tempCurve[0][0];

    int32_t diff = y1 - y2;
    int32_t measDiff = y1 - res;
    int32_t diffX = x1 - x2;
    int32_t interpVal = x1 - ((measDiff * diffX) / diff );
    return (int16_t)(interpVal);
}

uint16_t calculateTargetOutputVoltageFromTemp(int16_t temp)
{
    /* This function will take the temperature and return a PWM count with a linear temp range described in the defines at the top of this file */
    if (temp <= SYS_PWM_MIN_TARGET_TEMP_F)
        return SYS_PWM_MIN_TARGET_VOLTAGE_MV;

    if (temp >= SYS_PWM_MAX_TARGET_TEMP_F)
        return SYS_PWM_MAX_TARGET_VOLTAGE_MV;

    int32_t tempRange = SYS_PWM_MAX_TARGET_TEMP_F - SYS_PWM_MIN_TARGET_TEMP_F;

    int32_t targetmv = ((temp - SYS_PWM_MIN_TARGET_TEMP_F)*SYS_PWM_MAX_TARGET_VOLTAGE_MV)/tempRange;

    if (targetmv >= SYS_PWM_MAX_TARGET_VOLTAGE_MV)
        return SYS_PWM_MAX_TARGET_VOLTAGE_MV;
    if (targetmv <= SYS_PWM_MIN_TARGET_VOLTAGE_MV)
        return SYS_PWM_MIN_TARGET_VOLTAGE_MV;

    return (uint16_t)targetmv;

}

uint16_t calculatePWMDutyCycleFromTargetOutputVoltage(uint16_t targetmv)
{
    uint16_t mv = targetmv;
    if (mv > SYS_PWM_MAX_TARGET_VOLTAGE_MV)
        mv = SYS_PWM_MAX_TARGET_VOLTAGE_MV;
#if SYS_PWM_MIN_TARGET_VOLTAGE_MV > 0
    else if (mv < SYS_PWM_MIN_TARGET_VOLTAGE_MV)
        mv = SYS_PWM_MIN_TARGET_VOLTAGE_MV;
#endif
    /* This function will return the PWM cycle count needed to get close to the target voltage */
    uint32_t math = (mv * SYS_PWM_MAX_VALUE) / SYS_VCC_VOLTAGE_MV;
    if (math >= SYS_PWM_MAX_VALUE)
        return SYS_PWM_MAX_VALUE;

    return (uint16_t)math;
}

void initGPIOs(void)
{
    /* Set all pins to input with weak pull up resistor */
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN0);      // GPIO P1.0 gets a pull up since its connected to supply.
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);

    /* Configure ADC pins P1.3 and P1.4 as ADC inputs, P1.0 as VREF+ and P1.2 as VREF- */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN4, GPIO_TERNARY_MODULE_FUNCTION);
    //GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4, GPIO_TERNARY_MODULE_FUNCTION);

    /* Configure PWM outputs P1.6 and P1.7 as timer module outputs*/
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN7, GPIO_SECONDARY_MODULE_FUNCTION);
}

void initClocks(void)
{
    __bis_SR_register(SCG0);                // disable FLL
    CSCTL3 |= SELREF__REFOCLK;              // Set REFO as FLL reference source
    CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_3;// DCOFTRIM=3, DCO Range = 8MHz
    CSCTL2 = FLLD_0 + 0xF3;                 // DCODIV = 8MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                // enable FLL


    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK; // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                            // default DCODIV as MCLK and SMCLK source

   P1DIR |= BIT0 | BIT1;                   // set ACLK and LED pin as output
   P1SEL1 |= BIT1;                         // set ACLK pin as second function

   PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
                                           // to activate previously configured port settings
}

void initTimers(void)
{
    /* Use TimerA0 as the timer that is used to start the ADC sample
     * Since there is only Timer B0.0 0.1 and 0.2, We use B0.1 and 0.2 as outputs
     * using Output mode 7: reset/set
     *
     * Input clock: 2 Mhz, divider is 1 = 2MHz
     */

    Timer_B_initContinuousModeParam param = { .clockSource = TIMER_B_CLOCKSOURCE_SMCLK,
                                              .clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_1,
                                              .startTimer = false,
                                              .timerClear = TIMER_B_DO_CLEAR
    };

    Timer_B_stop(TIMER_B0_BASE);                                                                            // Stop the timer
    Timer_B_initContinuousMode(TIMER_B0_BASE, &param);                                                      // Initialize timer
    Timer_B_selectCounterLength(TIMER_B0_BASE, TIMER_B_COUNTER_10BIT);                                      // Set timer at 10 bit
    Timer_B_selectLatchingGroup(TIMER_B0_BASE, TIMER_B_GROUP_NONE);                                         // Make sure CCR registers are not grouped
    /* Set the PWM outputs for both capture and compare to reset/set */
    Timer_B_setOutputMode(TIMER_B0_BASE, TIMER_B_CAPTURECOMPARE_REGISTER_1, TIMER_B_OUTPUTMODE_RESET_SET);  // Set output of TB0.1 = reset/set
    Timer_B_setOutputMode(TIMER_B0_BASE, TIMER_B_CAPTURECOMPARE_REGISTER_2, TIMER_B_OUTPUTMODE_RESET_SET);  // Set output of TB0.2 = reset/set

    /* Set outputs to 1 (as close to 0 as we can get) */
    Timer_B_setCompareValue(TIMER_B0_BASE, TIMER_B_CAPTURECOMPARE_REGISTER_1, 1);                           // Set output of TB0.1 to 1/1024 of 3.3 V = 3.2 mV
    Timer_B_setCompareValue(TIMER_B0_BASE, TIMER_B_CAPTURECOMPARE_REGISTER_2, 1);                           // Set output of TB0.2 to 1/1024 of 3.3 V = 3.2 mV

    Timer_B_clearTimerInterrupt(TIMER_B0_BASE);                                                             // Clear any interrupts that might be set
    Timer_B_enableInterrupt(TIMER_B0_BASE);                                                                 // Enable timer interrupts (when timer overflows)
}

void initADC(void)
{
    ADC_disable(ADC_BASE);
    ADC_disableConversions(ADC_BASE, true);
    ADC_init(ADC_BASE, ADC_SAMPLEHOLDSOURCE_SC, ADC_CLOCKSOURCE_ADCOSC, ADC_CLOCKDIVIDER_1);
    ADC_enable(ADC_BASE);
    ADC_setupSamplingTimer(ADC_BASE, ADC_CYCLEHOLD_16_CYCLES, false);
    ADC_configureMemory(ADC_BASE, ADC_INPUT_A0, ADC_VREFPOS_AVCC, ADC_VREFNEG_AVSS);
    ADC_clearInterrupt(ADC_BASE, ADC_OVERFLOW_INTERRUPT_FLAG | ADC_TIMEOVERFLOW_INTERRUPT_FLAG | ADC_ABOVETHRESHOLD_INTERRUPT_FLAG | ADC_BELOWTHRESHOLD_INTERRUPT_FLAG | ADC_INSIDEWINDOW_INTERRUPT_FLAG | ADC_COMPLETED_INTERRUPT_FLAG);
    ADC_enableInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT);
}

void startADCSample(uint8_t channelSelect)
{
    if (channelSelect == 0)
    {
        ADC_configureMemory(ADC_BASE, ADC_INPUT_A3, ADC_VREFPOS_AVCC, ADC_VREFNEG_AVSS);
    } else {
        ADC_configureMemory(ADC_BASE, ADC_INPUT_A4, ADC_VREFPOS_AVCC, ADC_VREFNEG_AVSS);
    }
    ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
}

void timerOverflowChecker(void)
{
    timerOverflowCount++;
    if (timerOverflowCount >= NUM_TIMER_OVERFLOWS_BETWEEN_SAMPLES)
    {
        // Then we want to start an ADC sample
        adcSampleInput1 != adcSampleInput1;         // Flip which sample input is done next
        startADCSample(adcSampleInput1);
        timerOverflowCount = 0;
    }
}

/* ADC interrupt routine */
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
{
    switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
    {
        case ADCIV_NONE:
            break;
        case ADCIV_ADCOVIFG:
            break;
        case ADCIV_ADCTOVIFG:
            break;
        case ADCIV_ADCHIIFG:
            break;
        case ADCIV_ADCLOIFG:
            break;
        case ADCIV_ADCINIFG:
            break;
        case ADCIV_ADCIFG:
            // Check if this channel has had the previous value used. Lazy semaphore
            if (adcNewSample & (1 << adcSampleInput1) == 0)
            {
                adcSample[adcSampleInput1] = ADCMEM0;
                adcNewSample |= (1 << adcSampleInput1);
            }
            ADCIFG = 0;
            break;                                           // Clear CPUOFF bit from 0(SR)
        default:
            break;
    }
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMERB0_B0_ISR(void)
{
    switch(__even_in_range(TB0IV,TB0IV_TBIFG))
    {
        case TB0IV_NONE:            // No interrupt
            break;
        case TB0IV_TBCCR1:          // CCR1 not used
            break;
        case TB0IV_TBCCR2:          // CCR2 not used
            break;
        case TB0IV_TBIFG:           // Overflow
            timerOverflowChecker();
            break;
        default:
            break;
    }
}

#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMERB0_B1_ISR(void)
{
    switch(__even_in_range(TB0IV,TB0IV_TBIFG))
    {
        case TB0IV_NONE:            // No interrupt
            break;
        case TB0IV_TBCCR1:          // CCR1 not used
            break;
        case TB0IV_TBCCR2:          // CCR2 not used
            break;
        case TB0IV_TBIFG:           // Overflow
            break;
        default:
            break;
    }
}
