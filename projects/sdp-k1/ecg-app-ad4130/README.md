# ECG App with the AD4130

The ECG App project goal is to control the [AD4130 Evaluation Board](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD4130-8.html) to acquire lead I ECG data sampled at 500Hz. The Evaluation Board is connected to a user using wet electrodes, and controlled by the SDP-K1. Both boards are connected via the arduino header.

## Introduction

- SPI Communication using the Arduino header
  - [SDP-K1](https://www.analog.com/media/en/technical-documentation/eval-board-schematic/eval-sdp-ck1z-rev-e-schematic.pdf):
    - PB3 -> SCK
    - PB4 -> MISO
    - PA7 -> MOSI
    - PA15 -> CS
  - AD4130:
    - 13 -> SCK
    - 12 -> MISO
    - 11 -> MOSI
    - 10 -> CS

- Timer (TIM3), APB1: 48MHz
  - Preescaler: 48000

## AD4130 Evaluation Board

- [Schematics](https://wiki.analog.com/_media/resources/eval/user-guides/ad4130-8/eval-ad4130-8wardz_schematic.pdf)
- [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad4130-8)
  - [Hardware Guide](https://wiki.analog.com/resources/eval/user-guides/ad4130-8/hardwareguide#set-up_procedures)
