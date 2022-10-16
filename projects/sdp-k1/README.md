# SDP-K1 Development on VS Code

This document explains the toolchain set up to program and debug a [SDP-K1](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/sdp-k1.html#eb-overview) development board (STM32F4 based) using Visual Studio Code IDE in macOS Big Sur (11.7).

## Pre-requisites

- Python 3. [This guide](https://docs.python-guide.org/starting/install3/osx/) explains how to install Pyhton 3 in macOS. MacOS comes with Python pre-installed, but it's Python 2.7, which is deprecated. In order to install Python 3 it is recommended to install GCC (which can be obtained installing [Xcode](https://developer.apple.com/xcode/)) first, and then installing it through [Homebrew](https://brew.sh/), a package manager for macOS.
- VS Code Extensions:
  - [C/C++](https://code.visualstudio.com/docs/languages/cpp)
  - [Makefile Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.makefile-tools)
  - [Cortex-Debug](https://github.com/Marus/cortex-debug/wiki), an extension that provides debugging capabilities for ARM Cortex-M devices such us the STM32F469NIH6. This extension has some requirements:
    - [ARM GNU Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain) (AArch32 bare-metal target (arm-none-eabi)), which is a community supported pre-built GNU compiler toolchain for Arm based CPUs that provides arm-none-eabi-gdb (debugger) and related tools. It supports CPUs based on the A, R and M profiles of the Arm architecture (including Cortex-A, Cortex-R, Cortex-M).
    - [pyOCD](https://pyocd.io/), which is a Python based tool and API for debugging, programming, and exploring Arm Cortex microcontrollers. Through both built-in support and CMSIS-Packs, pyOCD supports nearly every Cortex-M MCU that is available on the market. It´s a GDB server that supports the CMSIS-DAP debugger. The SDP-K1 uses a CMSIS DAPLink interface.
      - To see the available built-in targets the command `pyocd list --targets` can be run. Since the STM32F469NIH6 is not included within the built-in targets, run the command `pyocd pack install stm32f469nihx` to install the required CMSIS-Pack.

## SDP-K1

### Overview

- STM32F469NIH6 Cortex-M4 microcontroller
  - Core performance up to 180 MHz
  - 384 kB of internal random access memory (RAM)
  - 2 MB flash memory
    - 16 MB synchronous dynamic ram (SDRAM)
- Micron MT48LC4M32B2B5-6A XIT:L
- Arduino Uno headers
- 120-pin small foot print connector
  - Hirose FX8-120P-SV1
- STM32F469NIH6 processor peripherals exposed: SPI, QSPI, I2C, GPIO, timers, UART
- Supported in the Arm Mbed compiler

### Resources

- [Schematic](https://www.analog.com/media/en/technical-documentation/eval-board-schematic/eval-sdp-ck1z-rev-e-schematic.pdf)
- [SDP-K1 User Guide](https://www.analog.com/media/en/technical-documentation/user-guides/EVAL-SDP-CK1Z-UG-1539.pdf)
- [SDP-K1 MBED Page](https://os.mbed.com/platforms/SDP_K1/)
  - To find the device name  (Mac OS X) use the command _ls /dev/tty.usbmodem*_
- [MBED User Guide](https://wiki.analog.com/resources/tools-software/mbed)