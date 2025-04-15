# Quadcopter_HTWG Firmware
 
## Overview

This repository contains the firmware for a quadcopter project, developed at HTWG Konstanz. The firmware is designed for a Texas Instruments Tiva TM4C123GXL LaunchPad (ARM Cortex-M4F based microcontroller) and utilizes the FreeRTOS real-time operating system (V9.0.0rc1) for task management.

The core functionality revolves around flight stabilization using sensor data from a 9-axis IMU (likely MPU925x [cite: 42]) fused with the Madgwick AHRS algorithm[cite: 46]. PID controllers are implemented for roll, pitch, and yaw control[cite: 57]. Input is primarily handled via a CPPM-based remote control, and status/debug information is displayed on an OLED screen using the u8glib library.

## Key Features

* **Real-time Control:** Uses FreeRTOS for managing tasks like flight control, sensor reading, and communication.
* **Sensor Fusion:** Integrates Gyroscope, Accelerometer, and Magnetometer data using the Madgwick AHRS algorithm and quaternion math for orientation estimation.
* **PID Stabilization:** Implements PID controllers for roll, pitch, and yaw stabilization.
* **Motor Control:** Drives quadcopter motors based on control algorithm outputs.
* **Remote Control:** Receives flight commands via a CPPM remote control signal.
* **Display Interface:** Outputs information to an OLED display (likely SSD1306 based) using the u8glib library.
* **Fault Management:** Includes a system for detecting and handling faults.
* **Debug Interface:** Provides a UART interface for debugging and potentially tuning parameters (like PID gains).
* **Utilities:** Contains various helper modules for math operations, delays, signal edge counting, linked function lists, and optional workload estimation.

## Hardware

* **Microcontroller:** Texas Instruments Tiva TM4C123GXL LaunchPad (ARM Cortex-M4F)
* **IMU Sensor:** 9-axis Gyro + Accelerometer + Magnetometer (Likely MPU925x)
* **Display:** OLED Display (Likely SSD1306 128x64)
* **Remote Control:** CPPM Receiver

## Software Architecture

The firmware follows a layered structure:

1.  **Setup:** Contains configuration files and microcontroller startup code.
2.  **Application Layer:** Houses the main application logic, including task management, flight task, receiver task, command task, and flight control algorithms.
3.  **Driver Layer:** Provides hardware abstraction for sensors, motors, remote control, display, debug interface, DMA, and port expander. Includes sensor fusion components.
4.  **FreeRTOS:** Contains the FreeRTOS kernel (V9.0.0rc1) source files, portable layer for ARM CM4F, and header files.
5.  **Utilities:** Includes helper modules for math, delays, fault handling, edge counting, linked function lists, and workload estimation.

## Configuration

* **`qc_setup.h`:** Main configuration file to enable/disable software modules and hardware features.
* **`FreeRTOSConfig.h`:** Standard FreeRTOS configuration options.
* **`peripheral_setup.h`:** Defines specific pins and peripherals used for the hardware.
* **`prioritys.h`:** Configures task and interrupt priorities.

## Dependencies

* **FreeRTOS V9.0.0rc1:** Included in the project structure.
* **u8glib:** Library for driving the OLED display, included within the display driver source.

## Building

The project is configured for Texas Instruments' Code Composer Studio (CCS).


## Derived from earlier version by
* Tobias Grimm 
* Daniel Eckstein

The FreeRTOS kernel components are licensed under a modified GPL, as detailed in `QuadCopter_HTWG/04_free_rtos/License/license.txt`.
