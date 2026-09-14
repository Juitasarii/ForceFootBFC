# ForceFootBFC

ForceFootBFC is an STM32-based embedded force sensing system developed for humanoid soccer robot applications.

This project focuses on:

* Loadcell data acquisition
* Real-time force sensing
* Center of Pressure (COP) calculation
* Embedded slave communication
* Robot balance monitoring

The system is designed using STM32 microcontrollers and multiple loadcell sensors to support humanoid robot foot pressure analysis and stability control.

---

# Features

* Multi-loadcell sensor reading
* Real-time force monitoring
* COP (Center of Pressure) calculation
* Sensor filtering and stabilization
* UART slave communication
* STM32 embedded implementation
* Real-time data processing

---

# Hardware Specifications

* STM32F401CC
* HX711 Loadcell Amplifier
* Multiple Loadcell Sensors
* UART Communication Interface
* Humanoid Robot Foot Platform

---

# Software & Tools

* STM32CubeIDE
* STM32 HAL Driver
* C Programming Language
* GitHub Desktop

---

# System Architecture

```text
Loadcell Sensors
        ↓
HX711 Amplifier
        ↓
STM32F401CC
        ↓
Filtering & COP Calculation
        ↓
UART Communication
        ↓
Master Controller
```
---

# Main Functions

## Loadcell Reading

Reads force values from multiple loadcell sensors through HX711 modules.

## Sensor Filtering

Reduces sensor noise and stabilizes force measurements.

## COP Calculation

Calculates Center of Pressure based on load distribution from multiple sensors.

## UART Communication

Transfers processed sensor data from STM32 slave device to the master controller.

---

# Future Development

* ROS2 integration
* Real-time visualization system
* Advanced balancing algorithm

---

# Notes

This project is intended for embedded systems and humanoid robotics research purposes.

---

# Author

Developed by Juita Sari
