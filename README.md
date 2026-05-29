# ForceFootBFC

ForceFootBFC is a force sensing and loadcell-based embedded system project developed for humanoid soccer robot applications.
This repository contains STM32 slave programs, master programs, sensor processing, communication systems, and force analysis algorithms used for robot balancing and movement control.

---

## Features

* Loadcell sensor reading
* Center of Pressure (COP) calculation
* Sensor filtering system
* UART/I2C/SPI communication
* STM32 embedded implementation
* Real-time force monitoring

---

## Hardware

* STM32F401CC
* HX711 Loadcell Amplifier
* Loadcell Sensors
* UART Communication Interface
* Humanoid Robot Platform

---

## Software

* STM32CubeIDE
* C Programming Language
* HAL Driver
* GitHub Desktop

---

## Project Structure

```text
ForceFootBFC/
├── STM32_Loadcell_Slave/
├── UART_Communication/
├── Sensor_Filtering/
├── COP_Calculation/
└── Documentation/
```

---

## Main Functions

### Loadcell Reading

Reads force values from multiple loadcell sensors using HX711 modules.

### Filtering

Applies filtering algorithms to reduce sensor noise and stabilize readings.

### COP Calculation

Calculates Center of Pressure based on sensor distribution.

### Communication

Transfers processed data to the master controller using serial communication.

---

## Status

Current Status:

* Stable Version
* Final STM32 Slave Program

---

## Author

Developed by Juita
Embedded System & Humanoid Robot Research
