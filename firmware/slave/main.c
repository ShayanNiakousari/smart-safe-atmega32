/* ============================================================
 * Smart Safe with Remote Alert - SLAVE UNIT (Remote Alarm)
 * ATmega32 @ 1MHz (Proteus default-compatible)
 * Single-file build. No delay.h library used - Timer0 systick.
 * ============================================================ */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>

#define F_CPU 1000000UL

/* ================================================================
 * SECTION 1: PIN / CONFIG DEFINITIONS
 * ================================================================ */

/* ---- USART ----
 * Must match Master's baud rate exactly: 9600
 * Double-speed USART (U2X=1): UBRR = 12 at F_CPU=1MHz for ~9600 baud
 */
#define USART_BAUD      9600
#define USART_UBRR      12

/* ---- LCD (LCD2) ----
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

/* ---- LED (D3, red): PD3 ---- */
#define LED_PORT        PORTD
#define LED_DDR         DDRD
#define LED_RED         PD3

/* ---- Buzzer (BUZ2): PC0 ---- */
#define BUZZER_PORT     PORTC
#define BUZZER_DDR      DDRC
#define BUZZER_BIT      PC0

/* ---- Timing ---- */
#define ALARM_DURATION_MS   5000UL

/* ---- Message buffer ---- */
#define MSG_BUF_SIZE     40

/* ================================================================
 * SECTION 2: SYSTICK (Timer0) - millis() / delay_ms() without delay.h
 * ================================================================ */

static volatile uint32_t g_millis = 0;

static void systick_init(void)
{
    TCCR0 = (1 << WGM01) | (1 << CS01); /* prescaler = 8 */
    OCR0  = 124;   /* 125,000Hz / 125 = 1000Hz -> 1ms tick */
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
 * SECTION 4: USART DRIVER (interrupt-driven RX ring buffer)
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
 * SECTION 5: LED / BUZZER helpers
 * ================================================================ */

static void led_init(void)
{
    LED_DDR |= (1 << LED_RED);
    LED_PORT &= ~(1 << LED_RED);
}

static void buzzer_init(void)
{
    BUZZER_DDR |= (1 << BUZZER_BIT);
    BUZZER_PORT &= ~(1 << BUZZER_BIT);
}

static void alarm_on(void)
{
    LED_PORT |= (1 << LED_RED);
    BUZZER_PORT |= (1 << BUZZER_BIT);
}

static void alarm_off(void)
{
    LED_PORT &= ~(1 << LED_RED);
    BUZZER_PORT &= ~(1 << BUZZER_BIT);
}

/* ================================================================
 * SECTION 6: MESSAGE HANDLING
 * ================================================================ */

static uint8_t contains(const char *haystack, const char *needle)
{
    return strstr(haystack, needle) != NULL;
}

static char msg_buf[MSG_BUF_SIZE];
static uint8_t msg_len = 0;

static void show_message_on_lcd(const char *msg)
{
    lcd_clear();
    lcd_gotoxy(0, 0);

    /* first 16 chars on line 1, next 16 on line 2 (LM016L = 16x2) */
    uint8_t len = strlen(msg);
    for (uint8_t i = 0; i < len && i < 16; i++) {
        lcd_data(msg[i]);
    }
    if (len > 16) {
        lcd_gotoxy(0, 1);
        for (uint8_t i = 16; i < len && i < 32; i++) {
            lcd_data(msg[i]);
        }
    }
}

/* ================================================================
 * SECTION 7: MAIN
 * ================================================================ */

int main(void)
{
    systick_init();
    sei();  /* Timer0 interrupt must run before lcd_init() uses delay_ms(). */
    led_init();
    buzzer_init();
    lcd_init();
    usart_init();

    lcd_gotoxy(0, 0);
    lcd_string("Remote Alarm");
    lcd_gotoxy(0, 1);
    lcd_string("Waiting...");

    uint32_t alarm_start = 0;
    uint8_t alarm_active = 0;

    while (1) {
        /* ---- receive bytes and assemble a line terminated by \n ---- */
        while (usart_available()) {
            char c = usart_read_char();

            if (c == '\n' || c == '\r') {
                if (msg_len > 0) {
                    msg_buf[msg_len] = '\0';

                    /* Display clear, human-readable status on the remote LCD. */
                    if (contains(msg_buf, "LOCKOUT MAX ATTEMPTS")) {
                        lcd_clear();
                        lcd_gotoxy(0, 0);
                        lcd_string("ALERT: LOCKOUT");
                        lcd_gotoxy(0, 1);
                        lcd_string("3 WRONG ATTEMPTS");
                        alarm_on();
                        alarm_active = 1;
                        alarm_start = millis();
                    } else if (contains(msg_buf, "LOCKOUT FINISHED")) {
                        alarm_off();
                        alarm_active = 0;
                        lcd_clear();
                        lcd_gotoxy(0, 0);
                        lcd_string("Remote Alarm");
                        lcd_gotoxy(0, 1);
                        lcd_string("Waiting...");
                    } else if (contains(msg_buf, "TAMPER")) {
                        lcd_clear();
                        lcd_gotoxy(0, 0);
                        lcd_string("ALERT: TAMPER");
                        lcd_gotoxy(0, 1);
                        lcd_string("CHECK SAFE");
                        alarm_on();
                        alarm_active = 1;
                        alarm_start = millis();
                    } else if (contains(msg_buf, "POWER FAIL") ||
                               contains(msg_buf, "FAILURE") ||
                               contains(msg_buf, "BREACH")) {
                        show_message_on_lcd(msg_buf);
                        alarm_on();
                        alarm_active = 1;
                        alarm_start = millis();
                    } else if (contains(msg_buf, "POWER RESTORED")) {
                        alarm_off();
                        alarm_active = 0;
                        lcd_clear();
                        lcd_gotoxy(0, 0);
                        lcd_string("POWER RESTORED");
                        lcd_gotoxy(0, 1);
                        lcd_string("SYSTEM NORMAL");
                    } else {
                        show_message_on_lcd(msg_buf);
                    }

                    msg_len = 0;
                }
            } else {
                if (msg_len < (MSG_BUF_SIZE - 1)) {
                    msg_buf[msg_len++] = c;
                }
            }
        }

        /* ---- auto-clear the alarm after its fixed duration ---- */
        if (alarm_active &&
            (uint32_t)(millis() - alarm_start) >= ALARM_DURATION_MS) {
            alarm_off();
            alarm_active = 0;
        }
    }

    return 0;
}

