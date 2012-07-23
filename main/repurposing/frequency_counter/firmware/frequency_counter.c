
    //  Pegasus Hub Controller Firmware
    //  ===============================
    //
    //  (c) Copyright L-3 Nautronix 2012
    //
    //     Version: 1.00a
    //      Author: Tim
    //
    // Description: This file contains all the firmware that resides in the
    //              small 8 bit AVR micro-controller on the Hub Controller
    //              printed circuit board. I have endevoured to keep all the
    //              code in one file rather than lots of libraries in an
    //              attempt to keep it easier to understand and simpler.
    //
    //              The code can be built with either the IAR compiler OR
    //              GCC and the two projects are squirreled away here.
    //              In the past slightly cleaner methods of hiding the
    //              differences between the two compilers have been used
    //              but haven't bothered here...
    //
    //              The code really only does two things:
    //                  1) a heartbeat signal to the sensors that encodes
    //                     the battery voltage into the signal
    //                  2) a switch controlled pulse extender for the
    //                     delayed firing of the inflater
    //
    //              Nearly all the meat of the code happens in the interrupt
    //              service routines as timing is fairly critical in here.
    //              Having said that the whole thing runs very slowly from
    //              the internal 1MHz RC oscillator in an attempt to preserve
    //              battery power.

//-----------------------------------------------------------------------------

#ifdef __IAR__

    #include <ioavr.h>
    #include <inavr.h>
    #define enableInterrupts __enable_interrupt
    #define sleep_cpu __sleep
    #define sleep_enable()

#else // AVR flavor of GCC

    #include <avr/io.h>
    #include <avr/interrupt.h>
    #include <avr/sleep.h>

    #define enableInterrupts() sei()
    #define disableInterrupts() cli()

#endif

//-----------------------------------------------------------------------------

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned long  u32;

typedef void (*state)(void);

#define BIT(X)  (1 << (X))
#define LOW(X)  ((X) & 0xff)
#define HIGH(X) (((X) >> 8) & 0xff)

//-----------------------------------------------------------------------------

    // Pin Assignments

#define g_inflator1_pin  PB3
#define g_inflator2_pin  PB2
#define g_status1_pin    PB0
#define g_status2_pin    PB1

#define g_sw1_1_pin      PA5
#define g_sw1_2_pin      PA7
#define g_sw1_4_pin      PA6
#define g_sw1_8_pin      PA4

#define g_sw2_1_pin      PB6
#define g_sw2_2_pin      PB5
#define g_sw2_4_pin      PB7
#define g_sw2_8_pin      PB4

#define g_bat1_sense_pin PA3
#define g_bat2_sense_pin PA1
#define g_bat3_sense_pin PA2
#define g_fire_sense_pin PA0

//-----------------------------------------------------------------------------

    // local, global variables

static state g_current_state;   // where we are in the state machine!
static u16 g_heartbeat_time = 0;
static u16 g_adc_time = 0;
#define HEARTBEAT_STEP (60000)
static u8 g_wait_count;
static u8 g_status_channel;

//-----------------------------------------------------------------------------

static void stateOffGap();    // need function prototype for this one...

//-----------------------------------------------------------------------------

static void
stateOn2(void)
{
    if (g_status_channel & 1)
        PORTB |= BIT(g_status1_pin);
    else
        PORTB |= BIT(g_status2_pin);

    // get the ADC value and work out when we need to go off!

    u16 adcv = ADCL;
    adcv    += (ADCH & 0x3) * 0x100;
    adcv    *= 3;
    adcv    += 200;

    ADCSRA = 0;         // turn OFF ADC to save some power

    g_adc_time = g_adc_time + adcv;
    OCR0B  = HIGH(g_adc_time);
    OCR0A  = LOW(g_adc_time);               // NOTE: low must be last!!!

    g_wait_count    = 5;
    g_current_state = stateOffGap;
}

//-----------------------------------------------------------------------------

static void
stateOff1(void)
{
    PORTB &= ~( BIT(g_status1_pin) | BIT(g_status2_pin) );

    // get the ADC value and work out when we need to go off!

    u16 adcv = ADCL;
    adcv    += (ADCH & 0x3) * 0x100;
    adcv    *= 3;
    adcv    += 200;

    ADMUX  = BIT(MUX1) | BIT(MUX0);
    ADCSRA = BIT(ADEN) | BIT(ADSC) | BIT(ADPS1);

    g_adc_time = g_adc_time + adcv;
    OCR0B  = HIGH(g_adc_time);
    OCR0A  = LOW(g_adc_time);               // NOTE: low must be last!!!

    g_current_state = stateOn2;
}

//-----------------------------------------------------------------------------

static void
stateOn1(void)
{
    if (g_status_channel & 1)
        PORTB |= BIT(g_status1_pin);
    else
        PORTB |= BIT(g_status2_pin);

    // get the ADC value and work out when we need to go off!

    u16 adcv = ADCL;
    adcv    += (ADCH & 0x3) * 0x100;
    adcv    *= 3;
    adcv    += 200;

    ADMUX  = BIT(MUX1);
    ADCSRA = BIT(ADEN) | BIT(ADSC) | BIT(ADPS1);

    g_adc_time = g_heartbeat_time + adcv;
    OCR0B  = HIGH(g_adc_time);
    OCR0A  = LOW(g_adc_time);               // NOTE: low must be last!!!

    g_current_state = stateOff1;
}

//-----------------------------------------------------------------------------

static void
stateOffGap(void)
{
    PORTB &= ~( BIT(g_status1_pin) | BIT(g_status2_pin) );

    g_heartbeat_time += HEARTBEAT_STEP;
    OCR0B  = HIGH(g_heartbeat_time);
    OCR0A  = LOW(g_heartbeat_time);     // NOTE: low must be last!!!

    // if this is the last one then

    g_wait_count--;
    if (g_wait_count == 0)
    {
        g_status_channel++;
        ADMUX  = BIT(MUX0);
        ADCSRA = BIT(ADEN) | BIT(ADSC) | BIT(ADPS1);
        g_current_state = stateOn1;
    }
}

//-----------------------------------------------------------------------------

    // timer 0 compare has gone off

#ifdef __IAR__
    #pragma vector = TIM0_COMPA_vect
    __interrupt void timer0CompareInterruptServiceRoutine(void)
#else
    SIGNAL(SIG_OUTPUT_COMPARE0A)
#endif
{
    g_current_state();
}

//-----------------------------------------------------------------------------

    // analog compare has gone off

#ifdef __IAR__
    #pragma vector = TIM0_COMPA_vect
    __interrupt void analogComparitorInterruptServiceRoutine(void)
#else
    SIGNAL(SIG_COMPARATOR)
#endif
{
    // need to put the whole delay thing into operation NOW!
}

//-----------------------------------------------------------------------------

int
main(void)
{
    // setup port directions

    DDRB = BIT(g_status1_pin) | BIT(g_status2_pin);

    // configure the Analog Comparator

//  ACSRA = BIT(ACI);

    // configure the Analog to Digital Converter

    DIDR0 = BIT(0) | BIT(1) | BIT(2) | BIT(3);
    ADMUX = BIT(MUX0);

    // configure timer 0

    TCCR0B = BIT(CS00);                 // no pre-scaling!
    TCCR0A = BIT(TCW0);                 // 16 bit mode
    OCR0B  = HIGH(g_heartbeat_time);
    OCR0A  = LOW(g_heartbeat_time);     // NOTE: low must be last!!!
    TIMSK  = BIT(OCIE0A);               // match

    // some values for the state machine

    g_wait_count    = 5;
    g_current_state = stateOffGap;      // start state machine in OFF state

    enableInterrupts();
    sleep_enable();

    while (1)
        sleep_cpu();
}
