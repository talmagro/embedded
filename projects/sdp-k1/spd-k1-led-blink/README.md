# SDP-K1 Development on VS Code

This document explains the toolchain set up to program and debug a SDP-K1 development board (STM32F4 based) using Visual Studio Code IDE in macOS Big Sur (11.7).

## Pre-requisites

- Python 3. [This guide](https://docs.python-guide.org/starting/install3/osx/) explains how to install Pyhton 3 in macOS. MacOS comes with Python pre-installed, But it's Python 2.7, which is deprecated. In order to install Python 3 it is recommended to install GCC (whoch can be obtained installing [Xcode](https://developer.apple.com/xcode/)) first, and then installing [Homebrew](https://brew.sh/), a package manager for macOS.
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html). This tool from STMicroelectronics allows an easy configuration of STM32 microcontrollers such us the STM32F469. It generates projects that contain all the firmware dependecies needed and initialization C code based on the configuration selected.
- VS Code Extensions:
  - [C/C++](https://code.visualstudio.com/docs/languages/cpp)
  - [Makefile Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.makefile-tools)
  - [Cortex-Debug](https://github.com/Marus/cortex-debug/wiki), an extension that provides debugging capabilities for ARM Cortex-M devices such us the STM32F469. This extension has some requirements:
    - [ARM GNU Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain) (AArch32 bare-metal target (arm-none-eabi)), which is a community supported pre-built GNU compiler toolchain for Arm based CPUs that provides arm-none-eabi-gdb (debugger) and related tools. The Arm GNU Toolchain, supports CPUs based on the A, R and M profiles of the Arm architecture (including Cortex-A, Cortex-R, Cortex-M).
    - [pyOCD GDB Server](https://pyocd.io/), which is a Python based tool and API for debugging, programming, and exploring Arm Cortex microcontrollers. It´s a GDB server that supports the CMSIS-DAP debugger. The SDP-K1 uses CMSIS-DAP.
