# FiubaSAT - OBC Programming for CubeSAT
FiubaSAT is a project aimed at developing the On-Board Computer (OBC) for a CubeSAT using the FreeRTOS real-time operating system and the libopencm3 library. This OBC will serve as the primary control system for the CubeSAT, managing various subsystems such as communication, power, sensors, and payload.

# Project Overview
The FiubaSAT OBC will be responsible for handling critical operations of the CubeSAT, such as:

- Managing communication between subsystems.
- Collecting and processing data from sensors.
- Controlling the satellite's power management.
- Executing commands received from the ground station.
- Managing payload operations.
- The project utilizes the following technologies:

FreeRTOS: A real-time operating system that ensures reliable task scheduling and resource management.
libopencm3: An open-source library for ARM Cortex-M microcontrollers, which provides peripheral drivers and low-level access to hardware.

# Table of Contents

- Project Overview
- Features
- Project Structure
- Getting Started
- How to Contribute
- License

# Features
- Real-time task scheduling using FreeRTOS.
- Multi-tasking and concurrent processes for handling different CubeSAT subsystems.
- Peripheral control using libopencm3, such as UART, I2C, SPI, and GPIO.
- Modular and scalable design, allowing easy expansion of features.
- Code structured to maximize power efficiency and minimize resource usage.

# Project Structure
FiubaSAT/
│
├── src/                   # Source code for the OBC
│   ├── main.c             # Entry point of the program
│   ├── tasks/             # FreeRTOS tasks
│   └── peripherals/       # Hardware abstraction layers using libopencm3
│
├── lib/                   # External libraries
│   ├── libopencm3/        # Low-level hardware access
│   └── FreeRTOS/          # Real-time operating system source code
│
├── docs/                  # Documentation for the project
│
├── tests/                 # Unit tests and integration tests
│
└── README.md              # Project readme file

# Getting Started
To get started with developing or running the OBC software for FiubaSAT, follow these instructions.

# Prerequisites
- Development Board: ARM Cortex-M based microcontroller (e.g., STM32, Bluepill, etc.)
- Toolchain: GCC for ARM or a compatible toolchain.
- FreeRTOS: Real-time operating system for task management.
- libopencm3: Low-level peripheral drivers for ARM Cortex-M microcontrollers.

# License
This project is licensed under the MIT License - see the LICENSE file for details.
