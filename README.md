# Embedded System Project.

This repository contains the firmware code for an embedded system project implemented on STM32F1 using the **Non-Preemptive Event-Triggered Scheduling, Fixed Prioritized Pre-emptive Scheduling with Time Slicing** model. The system integrates multiple peripherals such as AHT10 (temperature & humidity sensor), HD44780 LCD, UART communication, and PWM motor control.

## 📋 Project Description

The system performs the following tasks:
- **Task 1:** Read data from AHT10 sensor (temperature & humidity)
- **Task 2:** Display data on a 16x2 LCD
- **Task 3:** Transmit data via UART to PC using Hercules
- **Task 4:** Receive control commands from PC (start/stop motor, set speed, change direction)
- **Task 5:** Control motor speed and direction using PWM

Implemented in C for STM32 microcontrollers, using **STM32Cube HAL** libraries.

---

## ✨ Features

- 📡 **UART Communication:** Full-duplex communication with PC using Hercules terminal.
- 🌡️ **Sensor Integration:** Read real-time temperature and humidity from AHT10 sensor over I2C.
- 📺 **LCD Display:** Display current sensor readings and system state on a 16x2 LCD.
- 🔄 **Motor Control:** PWM control for DC motor speed and GPIO for direction.
- ⏱️ **Event-Triggered Scheduling:** Non-preemptive cooperative multitasking based on periodic timer interrupts.

---

## 🔧 Hardware Requirements

- STM32F103C8T6 ("Blue Pill")
- AHT10 Temperature & Humidity sensor (I2C)
- 16x2 LCD with HD44780 driver (4-bit mode)
- L298N Motor Driver Module
- DC Motor (12V)
- PC with Hercules or any Serial Terminal (baudrate: 115200)
- Power Supply (5V for logic, 12V for motor)

---

## 🚀 Getting Started
