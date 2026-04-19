# STM32 Snake Game (FreeRTOS)

This project implements a real-time Snake game on an STM32 microcontroller using FreeRTOS, with user input provided via a Python interface.

## System Overview
- The Snake game runs entirely on the STM32 microcontroller.
- An OLED display (SSD1306) connected to the microcontroller is used to render the game.
- A Python script running on a computer captures keyboard inputs and sends them to the STM32 via UART.

## Features
- Real-time task management using FreeRTOS
- UART-based communication between PC and microcontroller
- Keyboard-controlled gameplay via Python
- OLED display output

## Technologies
- C (STM32 HAL)
- FreeRTOS
- Python (for input handling)
- UART communication
- SSD1306 OLED display

## Description
The system is designed as a distributed setup where the game logic is executed on the embedded device, while user input is handled externally. The Python script reads keyboard inputs (W/A/S/D) and transmits them over a serial connection to the STM32, enabling real-time interaction with the game.

This project demonstrates embedded systems development, real-time programming, and hardware-software integration.
