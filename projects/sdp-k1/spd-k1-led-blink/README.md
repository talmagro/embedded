# SDP-K1 LED Blink

The SDP-K1 LED Blink project goal is to get familiarized with the SDP-K1 development platform (STM32F469NIH6 based), STM32CubeMX and the VS Code set up and toolchain for embedded C development.

## Introduction

The idea is to toggle the LED4 (DS3) every 500ms. In order to do so, the pin PK4 (D7), which controls the MOSFET (Q4) that can power the LED, is configured as a GPIO. The pin is toggled every 500ms using the HAL_Delay function. Timers will be used in future projects to optimeze delays and avoid blocking behaviours.
