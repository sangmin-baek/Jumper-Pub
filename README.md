# Jumping Robot Controller  

## Overview  
This project provides an embedded system for a jumping robot. The system consists of two hardware components:  
1. **Robot Controller** – Responsible for controlling the robot’s movements.  
2. **User Command Controller** – A wireless remote controller for user commands.  

## Hardware  

### 1. Robot Controller  
The robot controller manages movement control and sensor data processing. It consists of two MCUs:  
- One for **BLDC motor control**  
- Another for **high-level control** (data processing, sensor reading, RF transceiving)  

<img src="Jumper%20controller.jpeg" width="500">

#### **Hardware Components**  
- **MCUs**: Dual **STM32H7**  
- **BLDC Motor Driver**: **DRV8311**  
- **DC Motor Driver**: **HR8833**  
- **IMU**: **ISM330DHCX**  
- **RF Communication**: **HC-12** (Long-range wireless communication module)  
- **Encoder**: **AS5047P**  
- **Angle Sensor & ADC**: For precise position sensing  
- **File Handling & Storage**: **SDIO** (Secure Digital Input Output) for data logging  
- **Regulators**:  
  - **12V to 6V** → **TPSM863252**  
  - **6V to 3.3V** → **LP5912**  

---

### 2. User Command Controller  
The user command controller sends control signals to the robot via RF communication.  

<img src="User%20command%20controller.jpeg" width="500">

#### **Hardware Components**  
- **MCU**: **STM32F4 Nucleo**  
- **Input Interfaces**:  
  - **DIP switches** (for mode selection)  
  - **Tact switches** (for discrete commands)  
  - **Joystick** (for movement control)  
  - **Potentiometer** (for analog input adjustments)  
- **RF Communication**: **HC-12**  

---

## Software  

### Robot Controller  
The robot controller software is responsible for:  
- **BLDC Motor Position Control**: Field-Oriented Control (FOC)  
- **DC Motor Position Control**: Precise motor control for jumping maneuvers  
- **IMU Data Processing**: Reading and filtering IMU sensor data for stability  
- **Data Logging**: Storing sensor data and control states for analysis  

---
