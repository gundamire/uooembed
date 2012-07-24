
    //  Reticulation Controller MkII
    //  ============================
    //
    //      start date:   15th Dec 2010
    //      current date: 16th Dec 2010
    //      cpu:          ATMega48

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

//#include "avr_shared.h"
//#include "embedded_common.h"
//#include "network_transport\network_transport.h"
//#include <avr/eeprom.h>
//#include "serial\serial.h"
//#include <string.h>

//-----------------------------------------------------------------------------

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned long  u32;
typedef enum { false, true } bool;

typedef void (*state)(void);

#define BIT(X)  (1 << (X))
#define LOW(X)  ((X) & 0xff)
#define HIGH(X) (((X) >> 8) & 0xff)

//-----------------------------------------------------------------------------

    // pin definitions on port B

#define RX_LED_PIN     (0)  // comms LED
#define HB_LED_PIN     (1)  // heart-beat LED
#define SOLENOID_8_PIN (2)

    // pin definitions for port C

#define SOLENOID_2_PIN (0)
#define SOLENOID_7_PIN (1)
#define SOLENOID_6_PIN (2)
#define SOLENOID_5_PIN (3)
#define SOLENOID_4_PIN (4)
#define SOLENOID_3_PIN (5)

    // pin definitions in port D

#define RTC_CLK_PIN    (2)
#define RTC_CE_PIN     (3)
#define RTC_IO_PIN     (4)
#define SOLENOID_1_PIN (5)

//-----------------------------------------------------------------------------

#define NUM_WATERING_ENTRIES (150)

//-----------------------------------------------------------------------------

    // internal data types

typedef struct 
{
    unsigned m_start_time : 11;     // when to start in seconds since midnight
    unsigned m_duration   : 7;      // how long, in minutes, to run for
    unsigned m_solenoid   : 3;      // which of the 8 solenoids to drive
    unsigned m_day        : 3;      // which day of week we're in (0 = off)
} WateringEntry;

//-----------------------------------------------------------------------------

    // local data variables

static WateringEntry g_watering_info[NUM_WATERING_ENTRIES];
static u8            g_active_solenoid = 0;
static u8            g_minutes_remaining = 0;
static u8            g_last_error = 0;

    // local data constants

static const char INFO_STRING[] = "Retic V7.17d [" __DATE__ ", " __TIME__ "]";
static const u16  VERSION       = 717;      // v7.17

//-----------------------------------------------------------------------------
//---#######################################################################---
//-----------------------------------------------------------------------------

static void
setupHardware(void)
{
    MCUCR |= BIT(PUD);
    
    PORTB = BIT(RX_LED_PIN);    // start OFF
    DDRB |= BIT(HB_LED_PIN) |   // allow the LED to flash
            BIT(RX_LED_PIN) |
            BIT(SOLENOID_8_PIN); 
    
    PORTC = 0;
    DDRC  = BIT(SOLENOID_2_PIN) |
            BIT(SOLENOID_3_PIN) |
            BIT(SOLENOID_4_PIN) |
            BIT(SOLENOID_5_PIN) |
            BIT(SOLENOID_6_PIN) |
            BIT(SOLENOID_7_PIN); 
    
    PORTD = 0;
    DDRD |= BIT(RTC_CLK_PIN) | 
            BIT(RTC_CE_PIN)  | 
            BIT(RTC_IO_PIN)  | 
            BIT(SOLENOID_1_PIN);

    // setup the counter
    // setup the interrupt for the timer 0 and timer 1

//    TCCR0A = 0;                 // clocks off to start off with...
//    TCCR0B = 0;
//    TCCR1B = 0;
//    TIMSK0 = BIT(TOIE0);
}

//-----------------------------------------------------------------------------

static u8
toBCD(u8 x)
{
    u8 tv = 0;
    while (x > 10)
    {
        tv++;
        x -= 10;
    }
    return (tv << 4) | x;
}

//-----------------------------------------------------------------------------

static u8
fromBCD(u8 x)
{
    u8 rv = (x >> 4) * 10;
    rv += (x & 0xf);
    return rv;
}

//-----------------------------------------------------------------------------

static void
shortRTCDelay(void)
{
    volatile u8 xx;
    for (xx = 0; xx < 9; xx++)
        ;
}

static void
txRTC(u8 val)
{
    u8 idx;
    for (idx = 0; idx < 8; idx++)
    {
        if (val & 0x01)
            PORTD |= BIT(RTC_IO_PIN);
        else
            PORTD &= ~BIT(RTC_IO_PIN);
        shortRTCDelay();
        PORTD |= BIT(RTC_CLK_PIN);
        val >>= 1;
        shortRTCDelay();
        PORTD &= ~BIT(RTC_CLK_PIN);
    }
    PORTD &= ~BIT(RTC_IO_PIN);      // leave low...
}

static u8
rxRTC(void)
{
    u8 rv = 0;
    u8 idx;
    DDRD &= ~BIT(RTC_IO_PIN);
    DDRD &= ~BIT(RTC_IO_PIN);
    for (idx = 0; idx < 8; idx++)
    {
        rv >>= 1;
        shortRTCDelay();
        if (PIND & BIT(RTC_IO_PIN))
            rv |= 0x80;
        PORTD |= BIT(RTC_CLK_PIN);
        shortRTCDelay();
        PORTD &= ~BIT(RTC_CLK_PIN);
    }
    PORTD &= ~BIT(RTC_IO_PIN);
    DDRD |= BIT(RTC_IO_PIN);
    return rv;
}

    // read a memory location OR RTC register

static u8
rdRTCReg(u8 address)
{
    u8 temp = 0x81 | ((address & 0x3f) << 1);
    PORTD |= BIT(RTC_CE_PIN);
    shortRTCDelay();
    txRTC(temp);
    temp = rxRTC();
    shortRTCDelay();
    PORTD &= ~BIT(RTC_CE_PIN);
    return temp;
}

    // read a memory location OR RTC register

static void
wrRTCReg(u8 address, u8 data)
{
    u8 temp = 0x80 | ((address & 0x3f) << 1);
    PORTD |= BIT(RTC_CE_PIN);
    shortRTCDelay();
    txRTC(temp);
    txRTC(data);
    shortRTCDelay();
    PORTD &= ~BIT(RTC_CE_PIN);
}

//-----------------------------------------------------------------------------

static bool
checkRTC()
{
    if (rdRTCReg(42) != 'C')
    {
        g_last_error = 98;
        return false;
    }
    if (rdRTCReg(43) != 'x')
    {
        g_last_error = 99;
        return false;
    }
    if (rdRTCReg(8) != 0xa5)
    {
        g_last_error = 23;
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------

static void
configRTC(void)
{
    u8 idx;
    wrRTCReg(7, 0x00);          // disable the write protect for the moment!
    wrRTCReg(0, 0x80);          // leave clock OFF until valid!
    for (idx = 1; idx < 6; idx++)
        wrRTCReg(idx, 0);
    wrRTCReg(8, 0xa5);          // allow capacitor to charge up
    wrRTCReg(42, 'C');          // check value 1
    wrRTCReg(43, 'x');          // check value 2
    wrRTCReg(7, 0x80);          // write protect back on
}

//-----------------------------------------------------------------------------

static void
setupRTC(void)
{
    // setup the RTC

    rdRTCReg(0);                // couple of dummy reads to make sure all happy!
    rdRTCReg(0);
    
    if (!checkRTC())
        configRTC();
}

//-----------------------------------------------------------------------------
//#############################################################################
//-----------------------------------------------------------------------------

#define EEPROM_OFFSET      (5)
#define EEPROM_CHECK_ADDR1 (202)
#define EEPROM_CHECK_ADDR2 (EEPROM_CHECK_ADDR1 + 1)

static u8
rdEEPROM(u8 address)
{
    while (EECR & BIT(EEPE))    // wait for any previous write to complete
        ;
    EEARL = address + EEPROM_OFFSET;
    EEARH = 0;
    EECR  = BIT(EERE);
    return EEDR;
}

//-----------------------------------------------------------------------------

static void
wrEEPROM(u8 address, u8 data)
{
    while (EECR & BIT(EEPE))    // wait for previous write
        ;
    EEARL = address + EEPROM_OFFSET; // set address
    EEARH = 0;
    EEDR  = data;               //   ...and data
    u8 temp = SREG;
    disableInterrupts();
    EECR  = BIT(EEMPE);
    EECR  = BIT(EEPE);          // start write
    SREG = temp;                // restore interrupts
    while (EECR & BIT(EEPE))    // wait for write
        ;
    EEARH = 1;                  // move address to unused block when finished....
}

//-----------------------------------------------------------------------------

static bool
checkEEPROM(void)
{
    if (rdEEPROM(EEPROM_CHECK_ADDR1) != 'C')
    {
        g_last_error = 213;
        return false;
    }
    if (rdEEPROM(EEPROM_CHECK_ADDR2) != 'x')
    {
        g_last_error = 39;
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------

static void
configEEPROM(void)
{
    u8 idx;
    for (idx = 0; idx < 250; idx++)
        wrEEPROM(idx, 0);
    wrEEPROM(EEPROM_CHECK_ADDR1, 'C');
    wrEEPROM(EEPROM_CHECK_ADDR2, 'x');
}

//-----------------------------------------------------------------------------

static void
setupEEPROM(void)
{
    if (checkEEPROM())
    {
        u8 idx;
        u8* bp = (u8*)(&g_watering_info[0]);
        for (idx = 0; idx < sizeof(g_watering_info); idx++)
            *bp++ = rdEEPROM(idx);
    }
    else
    {
        configEEPROM();
    }
}

//-----------------------------------------------------------------------------

static void
updateEEPROM(void)
{
    u8 idx;
    u8* bp = (u8*)(&g_watering_info[0]);
    for (idx = 0; idx < sizeof(g_watering_info); idx++)
    {
        u8 btw = *bp++;
        wrEEPROM(idx, btw);
        u8 brb = rdEEPROM(idx);
        if (brb != btw)
            g_last_error = 199;
    }
}

//-----------------------------------------------------------------------------
//#############################################################################
//-----------------------------------------------------------------------------

    // we get occasional pings from the AVR to let us know that it is still
    // alive and stuff is working...

static void
ping(void)
{
    NetworkPacket ping_packet =
    { 
        BROADCAST_ADDRESS, 
        NETWORK_ADDRESS, 
        0x42, 
        COMMAND_PING, 
        0 
    };
    txSerialPacket(&ping_packet);
}

//-----------------------------------------------------------------------------
//-- ####################################################################### --
//-----------------------------------------------------------------------------

static void
solenoidsOff(void)
{
    PORTB &= ~(BIT(SOLENOID_8_PIN));
    PORTC &= ~(BIT(SOLENOID_2_PIN) | 
               BIT(SOLENOID_3_PIN) | 
               BIT(SOLENOID_4_PIN) | 
               BIT(SOLENOID_5_PIN) | 
               BIT(SOLENOID_6_PIN) | 
               BIT(SOLENOID_7_PIN));
    PORTD &= ~(BIT(SOLENOID_1_PIN));
}

//-----------------------------------------------------------------------------

static void
solenoidOn(u8 solnum)
{
    switch (solnum)
    {
        case 0:
            PORTD |= BIT(SOLENOID_1_PIN);
            break;
            
        case 1:
            PORTC |= BIT(SOLENOID_2_PIN);
            break;
            
        case 2:
            PORTC |= BIT(SOLENOID_3_PIN);
            break;
            
        case 3:
            PORTC |= BIT(SOLENOID_4_PIN);
            break;
            
        case 4:
            PORTC |= BIT(SOLENOID_5_PIN);
            break;
            
        case 5:
            PORTC |= BIT(SOLENOID_6_PIN);
            break;
            
        case 6:
            PORTC |= BIT(SOLENOID_7_PIN);
            break;
            
        case 7:
            PORTB |= BIT(SOLENOID_8_PIN);
            break;
            
        default:
            solenoidsOff();
            break;

    }            
}

//-----------------------------------------------------------------------------

static void
processPacket(NetworkPacket* packet)
{
    u8 temp            = packet->m_src_addr;
    packet->m_src_addr = packet->m_dst_addr;
    packet->m_dst_addr = temp;

    switch (packet->m_command)
    {
        case COMMAND_NOP :
            txSerialPacket(packet);
            packet->m_length = 0;
            break;

        case COMMAND_IDENT :
            strcpy(packet->m_data.m_string, INFO_STRING);
            packet->m_length = strlen(INFO_STRING);
            txSerialPacket(packet);
            break;

        case COMMAND_LOOPBACK :
            txSerialPacket(packet);
            break;

        case COMMAND_VERSION :
            packet->m_data.m_word[0] = VERSION;
            packet->m_length         = sizeof(VERSION);
            txSerialPacket(packet);
            break;

        case COMMAND_MAX_PACKET_SIZE :
            packet->m_data.m_byte[0] = NETWORK_PACKET_DATA_SIZE;
            packet->m_length         = sizeof(u8);
            txSerialPacket(packet);
            break;

        case COMMAND_SET_TIME :
            if (packet->m_length == 7)
            {
                wrRTCReg(7, 0x00);                            // disable the write protect for the moment!
                wrRTCReg(0, 0x80);                            // stop the clock while updating
                wrRTCReg(1, toBCD(packet->m_data.m_byte[1])); // minutes
                wrRTCReg(2, toBCD(packet->m_data.m_byte[2])); // hour -- 24 hour mode
                wrRTCReg(3, toBCD(packet->m_data.m_byte[3])); // day of month
                wrRTCReg(4, toBCD(packet->m_data.m_byte[4])); // month
                wrRTCReg(5, packet->m_data.m_byte[5] & 0x7);  // day of week
                wrRTCReg(6, toBCD(packet->m_data.m_byte[6])); // year
                wrRTCReg(0, toBCD(packet->m_data.m_byte[0])); // seconds -- turn clock back on as well...
                wrRTCReg(7, 0x80);                            // write protect back on
            }
            packet->m_length = 0;
            txSerialPacket(packet);
            break;
            
        case COMMAND_GET_TIME :
            packet->m_data.m_byte[0] = fromBCD(rdRTCReg(0));  // seconds
            packet->m_data.m_byte[1] = fromBCD(rdRTCReg(1));  // minutes
            packet->m_data.m_byte[2] = fromBCD(rdRTCReg(2));  // hours
            packet->m_data.m_byte[3] = fromBCD(rdRTCReg(3));  // day of month
            packet->m_data.m_byte[4] = fromBCD(rdRTCReg(4));  // month
            packet->m_data.m_byte[5] = rdRTCReg(5);           // day of week
            packet->m_data.m_byte[6] = fromBCD(rdRTCReg(6));  // year
            packet->m_length = 7;
            txSerialPacket(packet);
            break;

        case COMMAND_RETIC_SET_SOLENOID :
            {
                u8 soladr = packet->m_data.m_byte[0];
                if ( (packet->m_length == 6) && (soladr < NUM_WATERING_ENTRIES) )
                {
                    u16 tv = packet->m_data.m_byte[1] * 0x100 + packet->m_data.m_byte[2];
                    g_watering_info[soladr].m_start_time = tv;
                    g_watering_info[soladr].m_day        = packet->m_data.m_byte[3];
                    g_watering_info[soladr].m_duration   = packet->m_data.m_byte[4];
                    g_watering_info[soladr].m_solenoid   = packet->m_data.m_byte[5] - 1;
                    packet->m_length = 0;
                    updateEEPROM();
                }
                else
                {
                    networkError(packet, ERROR_BAD_SOLENOID_NUMBER);
                }
                txSerialPacket(packet);
            }
            break;

        case COMMAND_RETIC_GET_SOLENOID :
            if (packet->m_length != 1)
            {
                networkError(packet, ERROR_INSUFFICIENT_DATA);
            }
            else if (packet->m_data.m_byte[0] < NUM_WATERING_ENTRIES)
            {
                u8 soladr = packet->m_data.m_byte[0];
                u16 tv    = g_watering_info[soladr].m_start_time;
                packet->m_data.m_byte[0] = (tv >> 8) & 0xff;
                packet->m_data.m_byte[1] = tv & 0xff;
                packet->m_data.m_byte[2] = g_watering_info[soladr].m_day;
                packet->m_data.m_byte[3] = g_watering_info[soladr].m_duration;
                packet->m_data.m_byte[4] = g_watering_info[soladr].m_solenoid + 1;
                packet->m_length         = 5;
            }
            else
            {
                networkError(packet, ERROR_BAD_SOLENOID_NUMBER);
            }
            txSerialPacket(packet);
            break;
            
        case COMMAND_RETIC_GET_STATUS :
            packet->m_data.m_byte[0] = g_active_solenoid;    // which solenoid is currently running
            packet->m_data.m_byte[1] = g_minutes_remaining;  // how long left
            packet->m_data.m_byte[2] = g_last_error;         // possible last error
            packet->m_length = 3;
            txSerialPacket(packet);
            break;

        case COMMAND_RETIC_DRIVE_SOLENOID :
            if (packet->m_length == 2)
            {
                if (packet->m_data.m_byte[0] != g_active_solenoid)
                {
                    solenoidsOff();   
                    solenoidOn(g_active_solenoid - 1);
                }
                g_minutes_remaining = packet->m_data.m_byte[1];
                packet->m_length = 0;
            }
            else
            {
                networkError(packet, ERROR_INSUFFICIENT_DATA);
            }
            txSerialPacket(packet);
            break;

        default :
            networkError(packet, ERROR_UNKNOWN_COMMAND);
            txSerialPacket(packet);
            break;
    }
}

//-----------------------------------------------------------------------------

    // get a packet from the RS485 channel -- if we get a whole packet then
    // process it...

static void
processSerial(void)
{
    NetworkPacket* packet = rxSerialPacket();
    if (packet)
    {
        PORTB &= ~BIT(RX_LED_PIN);
        rdRTCReg(0);                // waste some time...
        processPacket(packet);
        PORTB |= BIT(RX_LED_PIN);
    }
}

//-----------------------------------------------------------------------------

    // see if we need to turn any solenoids ON or OFF

static void
processSolenoid(void)
{
    static u8 last_minute = 0xff;

    // if a solenoid is current ON then see if the time is up
    
    if (g_active_solenoid != 0)
    {
        if (g_minutes_remaining > 0)
        {
            u8 current_minute = fromBCD(rdRTCReg(1));
            if (last_minute != current_minute)
                g_minutes_remaining--;
            last_minute = current_minute;
        }
        if (g_minutes_remaining == 0)
        {
            solenoidsOff();
            g_active_solenoid = 0;
        }
        return;
    }
    
    u8 idx;
    u8 current_day = rdRTCReg(5) & 0x07;
    u16 min_in_day = fromBCD(rdRTCReg(2)) * 60 + fromBCD(rdRTCReg(1));
    for (idx = 0; idx < NUM_WATERING_ENTRIES; idx++)
    {
        if (g_watering_info[idx].m_day == current_day)
        {
            u16 mdt = g_watering_info[idx].m_start_time;
            if (mdt == min_in_day)
            {
                g_active_solenoid   = g_watering_info[idx].m_solenoid + 1;
                g_minutes_remaining = g_watering_info[idx].m_duration;
                solenoidOn(g_watering_info[idx].m_solenoid);
                last_minute = fromBCD(rdRTCReg(1));
            }
        }
    }
}

//-----------------------------------------------------------------------------

int
main(void)
{
    u8 current_seconds = 0;
    u8 last_seconds    = 0;
    
    // tweak the clock pre-scaler thingy...
    
    CLKPR = BIT(CLKPCE);        // sort out the division ratio
    CLKPR = 0;                  // sort out the division ratio

    // setup the hardware...

    setupSerialInterface(51);   // 9600 @ 8MHz
    setupHardware();
    setupEEPROM();
    setupRTC();
    enableInterrupts();
    
    // no spin forever effectively...
    
    while (true)
    {
        // have a look to see if we have been asked to do
        // anything by the serial port?

        processSerial();
        
        // now see if we have any solenoid activity to process!
        
        processSolenoid();

        // make a light flash so we know that something is happening...

        current_seconds = rdRTCReg(0);
        if (current_seconds != last_seconds)
        {
            PORTB ^= BIT(HB_LED_PIN);
            last_seconds = current_seconds;
        }
    }

    return 0;  // it will never get here -- to keep compiler quiet!
}
