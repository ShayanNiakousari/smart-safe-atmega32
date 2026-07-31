# 🔐 Smart Safe with Remote Alert

A dual-microcontroller smart safe system developed using **ATmega32** and **Embedded C**. The project provides secure password authentication, servo-based door locking, tamper detection, power-failure monitoring, EEPROM event logging, and communication between two microcontrollers via USART.

---

## ✨ Features

- Secure password authentication using a 4×4 keypad
- Servo motor-based electronic locking system
- Dual ATmega32 architecture (Master & Slave)
- USART communication between microcontrollers
- LCD status display
- Tamper detection using ADC
- External EEPROM event logging
- Software SPI implementation
- Power-failure detection
- Watchdog-based fault recovery
- Sleep mode for low-power operation
- Proteus simulation support

---

## 🛠 Hardware

- ATmega32 (Master)
- ATmega32 (Slave)
- 4×4 Matrix Keypad
- Servo Motor
- LCD 16×2
- External EEPROM
- LEDs
- Buzzer
- Potentiometer

---

## 💻 Technologies

- Embedded C
- AVR
- USART
- SPI
- EEPROM
- ADC
- PWM
- Timers
- Watchdog Timer
- Sleep Mode
- Proteus

---

## 🏗 System Architecture

The project consists of two ATmega32 microcontrollers.

### Master Unit
- Password authentication
- Keypad handling
- LCD interface
- Servo control
- Main system logic

### Slave Unit
- Remote alert management
- Event monitoring
- USART communication
- System supervision

---

## 🚀 How It Works

1. The user enters a password using the keypad.
2. The Master validates the password.
3. If the password is correct, the servo unlocks the safe.
4. Tamper and power-failure events are detected.
5. Events are stored in EEPROM.
6. The Master and Slave communicate via USART.

---

## 📂 Repository Structure

```text
firmware/
simulation/
docs/
images/
README.md
LICENSE
```

---

## 👨‍💻 Author

**Shayan Niakousari**

Computer Engineering Student

Software, Backend & Embedded Systems Developer
