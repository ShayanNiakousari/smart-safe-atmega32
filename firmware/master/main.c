/* ============================================================
 * Smart Safe with Remote Alert - MASTER UNIT
 * ATmega32 @ 1MHz (Proteus default-compatible)
 * Single-file build (all modules merged into one .c file)
 * No delay.h library used - all timing via Timer0 systick.
 * ============================================================ */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdio.h>

#define F_CPU 1000000UL

/* ================================================================
 * SECTION 1: PIN / CONFIG DEFINITIONS
 * ================================================================ */

/* ---- USART ----
 * Baud = 9600
 * Double-speed USART (U2X=1): UBRR = F_CPU/(8*BAUD)-1
 * = 1,000,000/(8*9600)-1 = 12.02 -> 12, actual baud 9615.4 (error ~0.16%)
 */
#define USART_BAUD      9600
#define USART_UBRR      12

/* ---- LCD (LCD1) ----
 * RS -> PA0 , RW -> PA1 , E -> PA2
 * D0..D7 -> PB0..PB7  (8-bit mode)
 */
#define LCD_CTRL_PORT   PORTA
#define LCD_CTRL_DDR    DDRA
#define LCD_RS          PA0
#define LCD_RW          PA1
#define LCD_E           PA2
#define LCD_DATA_PORT   PORTB
#define LCD_DATA_DDR    DDRB

/* ---- KEYPAD (4x4) ----
 * Rows (A,B,C,D)   -> PC0..PC3  (outputs, driven low one at a time)
 * Cols (1,2,3,4)   -> PC4..PC7  (inputs, pulled up)
 */
#define KEYPAD_PORT     PORTC
#define KEYPAD_DDR      DDRC
#define KEYPAD_PIN      PINC
#define KEYPAD_ROW0     PC0
#define KEYPAD_COL0     PC4

/* ---- EEPROM 25LC040 (software SPI) ----
 * SCK -> PD6, SI(MOSI) -> PD7, SO(MISO) -> PA5, CS -> PA7
 * NOTE: hardware SPI pins (PB5/6/7) are occupied by the LCD data
 * bus, so SPI to the EEPROM is bit-banged on these general pins.
 */
#define EE_PORT         PORTD
#define EE_DDR          DDRD
#define EE_SCK          PD6
#define EE_SI           PD7
#define EE_SO_PORT      PORTA
#define EE_SO_DDR       DDRA
#define EE_SO_PIN       PINA
#define EE_SO           PA5
#define EE_CS_PORT      PORTA
#define EE_CS_DDR       DDRA
#define EE_CS           PA7

/* ---- Tamper potentiometer: wiper -> PA3 (ADC3) ---- */
#define TAMPER_ADC_CHANNEL 3

/* ---- Power-fail divider: node -> PB3 (AIN1) ----
 * AIN0 (PB2) is internally replaced by the 1.22V bandgap (ACBG=1) */

/* ---- LEDs ---- */
#define LED_PORT        PORTD
#define LED_DDR         DDRD
#define LED_GREEN       PD4
#define LED_RED         PD3

/* ---- Wake button: INT0 -> PD2 ---- */
#define WAKE_DDR        DDRD
#define WAKE_PORT       PORTD
#define WAKE_PIN        PIND
#define WAKE_BIT        PD2

/* ---- Servo (door lock): OC1A -> PD5 ---- */
#define SERVO_DDR       DDRD
#define SERVO_BIT       PD5

/* ---- Buzzer: PA4 ---- */
#define BUZZER_PORT     PORTA
#define BUZZER_DDR      DDRA
#define BUZZER_BIT      PA4

/* ---- Timing constants ---- */
#define DOOR_OPEN_MS        5000UL
#define LOCKOUT_MS          30000UL
#define TAMPER_ALARM_MS     5000UL
#define IDLE_SLEEP_MS       10000UL
#define TAMPER_POLL_MS      100UL
#define TAMPER_THRESHOLD    15
#define MAX_FAILED_TRIES    3

/* ---- External EEPROM log memory map ---- */
#define EE_LOG_START        16
#define EE_LOG_RECORD_SIZE  6
#define EE_LOG_MAX_RECORDS  ((512 - EE_LOG_START) / EE_LOG_RECORD_SIZE)
#define EE_LOG_COUNT_ADDR   8

#define DEFAULT_PASSCODE    {1,2,3,4}

typedef enum {
    EVT_LOGIN_OK      = 1,
    EVT_LOGIN_FAIL     = 2,
    EVT_LOCKOUT        = 3,
    EVT_TAMPER         = 4,
    EVT_POWER_FAIL     = 5,
    EVT_POWER_RESTORED = 6,
    EVT_WATCHDOG_RESET = 7
} event_code_t;

/* ================================================================
 * SECTION 2: SYSTICK (Timer0) - millis() / delay_ms() without delay.h
 * ================================================================ */

static volatile uint32_t g_millis = 0;

static void systick_init(void)
{
    /* Timer0, CTC mode, prescaler=8 -> 125,000Hz timer clock
     * OCR0=124 -> compare match every 125 counts -> exactly 1ms */
    TCCR0 = (1 << WGM01) | (1 << CS01); /* prescaler = 8 */
    OCR0  = 124;
    TIMSK |= (1 << OCIE0);
    g_millis = 0;
}

ISR(TIMER0_COMP_vect)
{
    g_millis++;
}

static uint32_t millis(void)
{
    uint32_t val;
    uint8_t sreg = SREG;
    cli();
    val = g_millis;
    SREG = sreg;
    return val;
}

static void delay_ms(uint16_t ms)
{
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < ms) {
        /* busy wait driven by the Timer0 tick */
    }
}

/* ================================================================
 * SECTION 3: LCD DRIVER (8-bit mode)
 * ================================================================ */

static void lcd_pulse_enable(void)
{
    LCD_CTRL_PORT |= (1 << LCD_E);
    __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    LCD_CTRL_PORT &= ~(1 << LCD_E);
    __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
}

static void lcd_command(uint8_t cmd)
{
    LCD_CTRL_PORT &= ~(1 << LCD_RS);
    LCD_CTRL_PORT &= ~(1 << LCD_RW);
    LCD_DATA_PORT = cmd;
    lcd_pulse_enable();
    if (cmd == 0x01 || cmd == 0x02) {
        delay_ms(2);
    } else {
        delay_ms(1);
    }
}

static void lcd_data(uint8_t data)
{
    LCD_CTRL_PORT |= (1 << LCD_RS);
    LCD_CTRL_PORT &= ~(1 << LCD_RW);
    LCD_DATA_PORT = data;
    lcd_pulse_enable();
    delay_ms(1);
}

static void lcd_string(const char *str)
{
    while (*str) {
        lcd_data((uint8_t)*str++);
    }
}

static void lcd_clear(void)
{
    lcd_command(0x01);
}

static void lcd_gotoxy(uint8_t col, uint8_t row)
{
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
    lcd_command(addr);
}

static void lcd_init(void)
{
    LCD_CTRL_DDR |= (1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_E);
    LCD_DATA_DDR  = 0xFF;

    delay_ms(20);

    lcd_command(0x38);   /* 8-bit, 2-line, 5x7 */
    lcd_command(0x0C);   /* display ON, cursor/blink OFF */
    lcd_command(0x06);   /* increment, no shift */
    lcd_command(0x01);   /* clear */
    delay_ms(2);
}

/* ================================================================
 * SECTION 4: KEYPAD DRIVER (4x4 matrix)
 * ================================================================ */

static const char keymap[4][4] = {
    {'7', '8', '9', '/'},
    {'4', '5', '6', '*'},
    {'1', '2', '3', '-'},
    {'C', '0', '=', '+'}
};

static void keypad_init(void)
{
    KEYPAD_DDR  |= 0x0F;
    KEYPAD_DDR  &= ~0xF0;
    KEYPAD_PORT |= 0xF0;
    KEYPAD_PORT |= 0x0F;
}

static char keypad_scan(void)
{
    for (uint8_t row = 0; row < 4; row++) {
        KEYPAD_PORT |= 0x0F;
        KEYPAD_PORT &= ~(1 << (KEYPAD_ROW0 + row));

        __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n\t");

        uint8_t cols = KEYPAD_PIN & 0xF0;
        if (cols != 0xF0) {
            for (uint8_t col = 0; col < 4; col++) {
                if (!(cols & (1 << (KEYPAD_COL0 + col)))) {
                    KEYPAD_PORT |= 0x0F;
                    return keymap[row][col];
                }
            }
        }
    }
    KEYPAD_PORT |= 0x0F;
    return 0;
}

/* ================================================================
 * SECTION 5: USART DRIVER (interrupt-driven RX ring buffer)
 * ================================================================ */

#define RX_BUF_SIZE 32

static volatile char rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

static void usart_init(void)
{
    UBRRH = (uint8_t)(USART_UBRR >> 8);
    UBRRL = (uint8_t)(USART_UBRR);
    UCSRA = (1 << U2X);
    UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0); /* 8N1 */
}

ISR(USART_RXC_vect)
{
    uint8_t data = UDR;
    uint8_t next = (rx_head + 1) % RX_BUF_SIZE;
    if (next != rx_tail) {
        rx_buf[rx_head] = (char)data;
        rx_head = next;
    }
}

static void usart_send_char(char c)
{
    while (!(UCSRA & (1 << UDRE))) {
        /* wait for empty transmit buffer */
    }
    UDR = (uint8_t)c;
}

static void usart_send_string(const char *str)
{
    while (*str) {
        usart_send_char(*str++);
    }
}

static uint8_t usart_available(void)
{
    return rx_head != rx_tail;
}

static char usart_read_char(void)
{
    char c = 0;
    if (rx_head != rx_tail) {
        c = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    }
    return c;
}

/* ================================================================
 * SECTION 6: SOFTWARE SPI + EXTERNAL EEPROM (25LC040) + EVENT LOG
 * ================================================================ */

#define EE_CMD_WREN  0x06
#define EE_CMD_RDSR  0x05
#define EE_CMD_READ  0x03
#define EE_CMD_WRITE 0x02

static inline void ee_cs_low(void)  { EE_CS_PORT &= ~(1 << EE_CS); }
static inline void ee_cs_high(void) { EE_CS_PORT |=  (1 << EE_CS); }

static uint8_t ee_spi_transfer(uint8_t out)
{
    uint8_t in = 0;
    for (int8_t bit = 7; bit >= 0; bit--) {
        if (out & (1 << bit))
            EE_PORT |= (1 << EE_SI);
        else
            EE_PORT &= ~(1 << EE_SI);

        EE_PORT |= (1 << EE_SCK);
        __asm__ __volatile__ ("nop\n\tnop\n\t");

        in <<= 1;
        if (EE_SO_PIN & (1 << EE_SO)) in |= 1;

        EE_PORT &= ~(1 << EE_SCK);
        __asm__ __volatile__ ("nop\n\tnop\n\t");
    }
    return in;
}

static void ee_spi_init(void)
{
    EE_DDR |= (1 << EE_SCK) | (1 << EE_SI);
    EE_PORT &= ~(1 << EE_SCK);
    EE_SO_DDR &= ~(1 << EE_SO);
    EE_CS_DDR |= (1 << EE_CS);
    ee_cs_high();
}

static void ee_write_enable(void)
{
    ee_cs_low();
    ee_spi_transfer(EE_CMD_WREN);
    ee_cs_high();
}

static uint8_t ee_read_status(void)
{
    uint8_t sr;
    ee_cs_low();
    ee_spi_transfer(EE_CMD_RDSR);
    sr = ee_spi_transfer(0xFF);
    ee_cs_high();
    return sr;
}

static void ee_wait_ready(void)
{
    while (ee_read_status() & 0x01) {
        /* busy-wait for write cycle to finish (~5ms max) */
    }
}

static uint8_t ee_addr_msb_bit(uint16_t addr)
{
    return (addr & 0x100) ? 0x08 : 0x00;
}

static void ee_write_byte(uint16_t addr, uint8_t data)
{
    ee_write_enable();
    ee_cs_low();
    ee_spi_transfer(EE_CMD_WRITE | ee_addr_msb_bit(addr));
    ee_spi_transfer((uint8_t)(addr & 0xFF));
    ee_spi_transfer(data);
    ee_cs_high();
    ee_wait_ready();
}

static uint8_t ee_read_byte(uint16_t addr)
{
    uint8_t data;
    ee_cs_low();
    ee_spi_transfer(EE_CMD_READ | ee_addr_msb_bit(addr));
    ee_spi_transfer((uint8_t)(addr & 0xFF));
    data = ee_spi_transfer(0xFF);
    ee_cs_high();
    return data;
}

static uint8_t log_count = 0;

static void log_init(void)
{
    log_count = ee_read_byte(EE_LOG_COUNT_ADDR);
    if (log_count == 0xFF) {
        log_count = 0;
        ee_write_byte(EE_LOG_COUNT_ADDR, 0);
    }
}

static void log_event(event_code_t code, uint32_t timestamp, uint8_t extra)
{
    if (log_count >= EE_LOG_MAX_RECORDS) {
        log_count = 0;
    }

    uint16_t base = EE_LOG_START + (uint16_t)log_count * EE_LOG_RECORD_SIZE;

    ee_write_byte(base + 0, (uint8_t)code);
    ee_write_byte(base + 1, (uint8_t)(timestamp >> 24));
    ee_write_byte(base + 2, (uint8_t)(timestamp >> 16));
    ee_write_byte(base + 3, (uint8_t)(timestamp >> 8));
    ee_write_byte(base + 4, (uint8_t)(timestamp));
    ee_write_byte(base + 5, extra);

    log_count++;
    ee_write_byte(EE_LOG_COUNT_ADDR, log_count);
}

static const char *event_name(uint8_t code)
{
    switch (code) {
        case EVT_LOGIN_OK:       return "LOGIN_OK";
        case EVT_LOGIN_FAIL:     return "LOGIN_FAIL";
        case EVT_LOCKOUT:        return "LOCKOUT";
        case EVT_TAMPER:         return "TAMPER";
        case EVT_POWER_FAIL:     return "POWER_FAIL";
        case EVT_POWER_RESTORED: return "POWER_RESTORED";
        case EVT_WATCHDOG_RESET: return "WATCHDOG_RESET";
        default:                 return "UNKNOWN";
    }
}

static void log_dump_via_usart(void)
{
    char line[48];
    usart_send_string("---- LOG DUMP START ----\r\n");

    for (uint8_t i = 0; i < log_count; i++) {
        uint16_t base = EE_LOG_START + (uint16_t)i * EE_LOG_RECORD_SIZE;
        uint8_t code = ee_read_byte(base + 0);
        uint32_t ts = ((uint32_t)ee_read_byte(base + 1) << 24) |
                      ((uint32_t)ee_read_byte(base + 2) << 16) |
                      ((uint32_t)ee_read_byte(base + 3) << 8)  |
                      ((uint32_t)ee_read_byte(base + 4));
        uint8_t extra = ee_read_byte(base + 5);

        snprintf(line, sizeof(line), "%u,%s,t=%lu,d=%u\r\n",
                 i, event_name(code), (unsigned long)ts, extra);
        usart_send_string(line);
    }

    usart_send_string("---- LOG DUMP END ----\r\n");
}

/* ================================================================
 * SECTION 7: ADC (tamper potentiometer)
 * ================================================================ */

static void adc_init(void)
{
    /* AVCC reference, prescaler=64 -> 125kHz ADC clock (within 50-200kHz) */
    ADMUX  = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

static uint16_t adc_read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xE0) | (channel & 0x1F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {
        /* wait for conversion */
    }
    return ADC;
}

/* ================================================================
 * SECTION 8: POWER-FAIL DETECTION (Analog Comparator + bandgap ref)
 * ================================================================ */

static void power_fail_init(void)
{
    DDRB &= ~(1 << PB3);
    PORTB &= ~(1 << PB3);
    ACSR = (1 << ACBG);   /* AIN0 replaced by internal 1.22V bandgap */
}

static uint8_t power_fail_detected(void)
{
    /* ACO=1 when bandgap(1.22V) > AIN1(divider node), i.e. supply sagged */
    return (ACSR & (1 << ACO)) ? 1 : 0;
}

/* ================================================================
 * SECTION 9: SERVO PWM (Timer1, OC1A = PD5, 50Hz frame)
 * ================================================================ */

#define SERVO_TOP           2499
#define SERVO_PULSE_LOCK     125
#define SERVO_PULSE_UNLOCK   250

static void servo_init(void)
{
    SERVO_DDR |= (1 << SERVO_BIT);
    ICR1 = SERVO_TOP;
    /* Fast PWM, TOP=ICR1, non-inverting OC1A, prescaler=8; 1MHz/8=125kHz */
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    OCR1A = SERVO_PULSE_LOCK;
}

static void servo_lock(void)   { OCR1A = SERVO_PULSE_LOCK; }
static void servo_unlock(void) { OCR1A = SERVO_PULSE_UNLOCK; }

/* ================================================================
 * SECTION 10: WATCHDOG + SLEEP MODE
 * ================================================================ */

static uint8_t g_was_wdt_reset = 0;

static void power_mgmt_capture_reset_source(void)
{
    if (MCUCSR & (1 << WDRF)) {
        g_was_wdt_reset = 1;
    }
    MCUCSR = 0;
}

static void watchdog_init(void)
{
    wdt_enable(WDTO_2S);
}

static void watchdog_kick(void)
{
    wdt_reset();
}

ISR(INT0_vect)
{
    /* wakes the MCU from power-down; body intentionally empty */
}

static void wake_button_init(void)
{
    WAKE_DDR  &= ~(1 << WAKE_BIT);
    WAKE_PORT |=  (1 << WAKE_BIT);
    /* ATmega32 can wake from Power-down through INT0 only in low-level mode. */
    MCUCR &= ~((1 << ISC01) | (1 << ISC00));
    GIFR   = (1 << INTF0);   /* clear any pending INT0 flag */
    GICR  |= (1 << INT0);
}

static void enter_sleep_mode(void)
{
    wdt_disable();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    /* Atomic sleep sequence prevents an interrupt occurring between
       enabling sleep and executing SLEEP. */
    cli();
    GIFR = (1 << INTF0);
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();

    /* Disable INT0 until the next sleep request. This prevents repeated
       low-level interrupts while the wake button is still held down. */
    GICR &= ~(1 << INT0);

    while (!(WAKE_PIN & (1 << WAKE_BIT))) {
        /* wait until the active-low wake button is released */
    }

    watchdog_init();
}

/* ================================================================
 * SECTION 11: LEDs / BUZZER helpers
 * ================================================================ */

static void leds_init(void)
{
    LED_DDR |= (1 << LED_GREEN) | (1 << LED_RED);
    LED_PORT &= ~((1 << LED_GREEN) | (1 << LED_RED));
}

static void buzzer_init(void)
{
    BUZZER_DDR |= (1 << BUZZER_BIT);
    BUZZER_PORT &= ~(1 << BUZZER_BIT);
}

static void buzzer_on(void)  { BUZZER_PORT |=  (1 << BUZZER_BIT); }
static void buzzer_off(void) { BUZZER_PORT &= ~(1 << BUZZER_BIT); }

/* ================================================================
 * SECTION 12: PASSCODE / INTERNAL EEPROM
 * ================================================================ */

static uint8_t EEMEM ee_passcode[4] = DEFAULT_PASSCODE;
static uint8_t EEMEM ee_failcount   = 0;

static char entered[5];
static uint8_t entered_len = 0;

static void reset_entry(void)
{
    entered_len = 0;
    entered[0] = '\0';
    lcd_clear();
    lcd_gotoxy(0, 0);
    lcd_string("Enter code:");
    lcd_gotoxy(0, 1);
}

static uint8_t check_passcode(const char *digits)
{
    uint8_t stored[4];
    eeprom_read_block(stored, ee_passcode, 4);
    for (uint8_t i = 0; i < 4; i++) {
        if (stored[i] > 9) stored[i] = (uint8_t)(i + 1);
        if ((uint8_t)(digits[i] - '0') != stored[i]) {
            return 0;
        }
    }
    return 1;
}

static void passcode_eeprom_init(void)
{
    uint8_t stored[4];
    const uint8_t def[4] = {1, 2, 3, 4};
    uint8_t valid = 1;

    eeprom_read_block(stored, ee_passcode, 4);
    for (uint8_t i = 0; i < 4; i++) {
        if (stored[i] > 9) {
            valid = 0;
            break;
        }
    }

    /* Proteus often loads only the FLASH .hex and leaves EEPROM as 0xFF. */
    if (!valid) {
        eeprom_update_block(def, ee_passcode, 4);
    }

    uint8_t fails = eeprom_read_byte(&ee_failcount);
    if (fails == 0xFF || fails > MAX_FAILED_TRIES) {
        eeprom_update_byte(&ee_failcount, 0);
    }
}

static uint8_t get_fail_count(void)
{
    uint8_t v = eeprom_read_byte(&ee_failcount);
    return (v == 0xFF || v > MAX_FAILED_TRIES) ? 0 : v;
}
static void set_fail_count(uint8_t v) { eeprom_update_byte(&ee_failcount, v); }

/* ================================================================
 * SECTION 13: ALARM / ACCESS HANDLERS
 * ================================================================ */

static void trigger_tamper_alarm(void)
{
    LED_PORT |= (1 << LED_RED);
    buzzer_on();
    usart_send_string("ALERT:TAMPER DETECTED\r\n");
    log_event(EVT_TAMPER, (uint32_t)(millis() / 1000), 0);

    lcd_clear();
    lcd_gotoxy(0, 0);
    lcd_string("!! TAMPER !!");

    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < TAMPER_ALARM_MS) {
        watchdog_kick();
    }

    buzzer_off();
    LED_PORT &= ~(1 << LED_RED);
}

static void trigger_lockout(void)
{
    LED_PORT |= (1 << LED_RED);
    buzzer_on();
    usart_send_string("ALERT:LOCKOUT MAX ATTEMPTS\r\n");
    log_event(EVT_LOCKOUT, (uint32_t)(millis() / 1000), 0);

    /* A real 30-second countdown based on millis().
       The LCD displays 30, 29, ... 1, 0 and only then asks for the
       password again. Keypad input is ignored during the lockout. */
    uint32_t lock_start = millis();
    int8_t last_shown = -1;

    while (1) {
        uint32_t elapsed = (uint32_t)(millis() - lock_start);
        uint8_t remaining;

        if (elapsed >= LOCKOUT_MS) {
            remaining = 0;
        } else {
            remaining = (uint8_t)((LOCKOUT_MS - elapsed + 999UL) / 1000UL);
        }

        if ((int8_t)remaining != last_shown) {
            last_shown = (int8_t)remaining;

            lcd_clear();
            lcd_gotoxy(0, 0);
            lcd_string("LOCKED OUT");
            lcd_gotoxy(0, 1);
            lcd_string("Wait ");
            lcd_data((char)('0' + (remaining / 10)));
            lcd_data((char)('0' + (remaining % 10)));
            lcd_string("s");
        }

        watchdog_kick();

        if (remaining == 0) {
            /* Keep 00 visible briefly before returning to password entry. */
            uint32_t zero_start = millis();
            while ((uint32_t)(millis() - zero_start) < 700UL) {
                watchdog_kick();
            }
            break;
        }
    }

    buzzer_off();
    LED_PORT &= ~(1 << LED_RED);
    set_fail_count(0);
    entered_len = 0;
    entered[0] = '\0';

    usart_send_string("INFO:LOCKOUT FINISHED\r\n");
    reset_entry();
}

static void grant_access(void)
{
    LED_PORT |= (1 << LED_GREEN);
    usart_send_string("INFO:ACCESS GRANTED\r\n");
    log_event(EVT_LOGIN_OK, (uint32_t)(millis() / 1000), 0);

    lcd_clear();
    lcd_gotoxy(0, 0);
    lcd_string("Access Granted");

    servo_unlock();

    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < DOOR_OPEN_MS) {
        watchdog_kick();
    }

    servo_lock();
    LED_PORT &= ~(1 << LED_GREEN);
    set_fail_count(0);
}

static void deny_access(void)
{
    uint8_t fails = get_fail_count() + 1;
    set_fail_count(fails);

    usart_send_string("INFO:ACCESS DENIED\r\n");
    log_event(EVT_LOGIN_FAIL, (uint32_t)(millis() / 1000), fails);

    lcd_clear();
    lcd_gotoxy(0, 0);
    lcd_string("Wrong Code");
    delay_ms(1000);

    if (fails >= MAX_FAILED_TRIES) {
        trigger_lockout();
    }
}

/* ================================================================
 * SECTION 14: TAMPER POLLING
 * ================================================================ */

static uint16_t last_tamper_reading = 0;
static uint8_t tamper_baseline_set = 0;

static uint8_t check_tamper(void)
{
    uint16_t reading = adc_read(TAMPER_ADC_CHANNEL);

    if (!tamper_baseline_set) {
        last_tamper_reading = reading;
        tamper_baseline_set = 1;
        return 0;
    }

    int16_t delta = (int16_t)reading - (int16_t)last_tamper_reading;
    if (delta < 0) delta = -delta;

    last_tamper_reading = reading;

    return (delta > TAMPER_THRESHOLD) ? 1 : 0;
}

/* ================================================================
 * SECTION 15: POWER-FAIL MONITORING
 * ================================================================ */

static uint8_t power_fail_state = 0;

static void check_power_fail(void)
{
    uint8_t failed_now = power_fail_detected();

    if (failed_now && !power_fail_state) {
        power_fail_state = 1;
        usart_send_string("ALERT:POWER FAIL\r\n");
        log_event(EVT_POWER_FAIL, (uint32_t)(millis() / 1000), 0);
        LED_PORT |= (1 << LED_RED);
    } else if (!failed_now && power_fail_state) {
        power_fail_state = 0;
        usart_send_string("INFO:POWER RESTORED\r\n");
        log_event(EVT_POWER_RESTORED, (uint32_t)(millis() / 1000), 0);
        LED_PORT &= ~(1 << LED_RED);
    }
}

/* ================================================================
 * SECTION 16: USART COMMAND HANDLING (log dump request)
 * ================================================================ */

static void handle_usart_commands(void)
{
    if (usart_available()) {
        char c = usart_read_char();
        if (c == 'L' || c == 'l') {
            log_dump_via_usart();
        }
    }
}

/* ================================================================
 * SECTION 17: MAIN
 * ================================================================ */

int main(void)
{
    power_mgmt_capture_reset_source();

    systick_init();
    sei();  /* Timer0 interrupt must run before lcd_init() uses delay_ms(). */
    leds_init();
    buzzer_init();
    wake_button_init();
    lcd_init();
    keypad_init();
    usart_init();
    ee_spi_init();
    log_init();
    adc_init();
    /* power_fail_init() disabled: PB3 is already LCD data D3 in this wiring. */
    servo_init();
    passcode_eeprom_init();

    watchdog_init();

    usart_send_string("SYSTEM READY\r\n");
    usart_send_string("PASSCODE: 4 DIGITS\r\n");

    if (g_was_wdt_reset) {
        usart_send_string("WARN:RESET BY WATCHDOG\r\n");
        log_event(EVT_WATCHDOG_RESET, 0, 0);
    }

    reset_entry();

    uint32_t last_tamper_poll = millis();
    uint32_t last_activity = millis();

    while (1) {
        watchdog_kick();
        handle_usart_commands();
        /* check_power_fail() disabled because PB3 conflicts with LCD D3. */

        if ((uint32_t)(millis() - last_tamper_poll) >= TAMPER_POLL_MS) {
            last_tamper_poll = millis();
            if (check_tamper()) {
                trigger_tamper_alarm();
                reset_entry();
                last_activity = millis();
            }
        }

        char key = keypad_scan();
        if (key != 0) {
            last_activity = millis();

            while (keypad_scan() != 0) {
                watchdog_kick();
            }
            delay_ms(30);

            if (key == 'C') {
                reset_entry();
            } else if (key >= '0' && key <= '9') {
                if (entered_len < 4) {
                    entered[entered_len++] = key;
                    entered[entered_len] = '\0';
                    lcd_data('*');
                }

                if (entered_len == 4) {
                    delay_ms(200);
                    if (check_passcode(entered)) {
                        grant_access();
                        reset_entry();
                    } else {
                        deny_access();
                        /* trigger_lockout() already restores the entry screen
                           after the 30-second lockout. For ordinary failures,
                           restore it here. */
                        if (get_fail_count() != 0) {
                            reset_entry();
                        }
                    }
                    last_activity = millis();
                }
            }
        }

        if ((uint32_t)(millis() - last_activity) >= IDLE_SLEEP_MS) {
            usart_send_string("SYSTEM SLEEP\r\n");
            lcd_clear();
            lcd_command(0x08);

            wake_button_init();
            enter_sleep_mode();

            lcd_command(0x0C);
            lcd_clear();
            lcd_gotoxy(0, 0);
            lcd_string("System Awake");
            lcd_gotoxy(0, 1);
            lcd_string("Enter Password");
            usart_send_string("SYSTEM AWAKE\r\n");
            delay_ms(800);
            reset_entry();
            last_activity = millis();
        }
    }

    return 0;
}


