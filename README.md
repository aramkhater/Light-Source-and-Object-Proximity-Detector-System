# DCS Final Project - Object and Light Source Detection System

## Project Overview
This project focuses on implementing a system capable of detecting light sources and monitoring objects in space using the **MSP430G2553** microcontroller. It integrates a blend of sensors and actuators managed by the MCU to perform precise detection and location measurements. 

The firmware is developed primarily in **C**, with specific time-critical sections implemented in assembly. It employs a layered architecture—comprising **Board Support Package (BSP)**, **Hardware Abstraction Layer (HAL)**, **Application Programming Interface (API)**, and **Main Application logic**—to ensure modularity and maintainability.



## System Features
* **Timers and ADC:** Utilized for precise PWM generation, input capture for distance timing, and 10-bit analog-to-digital conversions for light sensing.
* **Interrupt-Driven Processing:** Enhances real-time capabilities by using interrupts for UART communication, pulse-width measurement, and button debouncing.
* **Serial Communication:** Employs UART (RS-232) to interface with a PC-side GUI at 9600 BPS.
* **Low Power Design:** Optimized for power efficiency using MSP430 Low Power Modes (LPM).

## Components Used
* **Ultrasonic Sensor (HC-SR04):** Detects objects by emitting sound waves and measuring the high-level pulse time of the echoes to calculate range (2cm–450cm).
* **LDR Sensors:** Dual Light Dependent Resistors measure light intensity and convert it to voltage to determine proximity to light sources within a 0.5m range.
* **Servo Motor:** Adjusts the angle of sensors across a 180° arc using PWM to facilitate spatial scanning.
* **LCD Display:** Provides local feedback, such as menu navigation and status updates.



## System Operations
<img width="286" height="410" alt="Main Menu" src="https://github.com/user-attachments/assets/f28831e2-879e-475b-8816-dc10f4a2345c" />

### Object Detection
Conducts a 180-degree scan to identify objects in space. The system calculates distance, angular position, and object width based on 180 individual samples, transmitting the results to the PC for dynamic visualization.
![OD](https://github.com/user-attachments/assets/b62d9c11-cd63-42a4-af0e-2e70159fc75a)
<img width="752" height="837" alt="Screenshot (17)" src="https://github.com/user-attachments/assets/e4015ce7-6285-4b09-a825-4676878e0875" />


### Telemeter - Dynamic Scanning Interface
Allows for angle-specific distance measurements. The user selects an angle via the PC GUI, and the servo points the sensors to that position to provide real-time distance data.

![Tele](https://github.com/user-attachments/assets/581c697f-b796-41e6-a29c-bd24743c8cc9)

<img width="472" height="400" alt="Telemeter" src="https://github.com/user-attachments/assets/87d0935e-7f84-492a-9cc9-db746eadde40" />

### Light Source Detection
Identifies light sources within a 180-degree scan using the dual LDR sensors. The system maps the light intensity and position to identify the location of light emitters.

![LIhtDet](https://github.com/user-attachments/assets/977cad97-3a92-4481-a6ce-95452148b4ac)

<img width="707" height="783" alt="LD" src="https://github.com/user-attachments/assets/6ebd3a11-8d5a-402a-9bcc-5d5aac0fe4fa" />

### Integrated Detection (Bonus)
Merges the functionalities of detecting both objects and light sources into a single 180-degree sweep, producing a combined output of detected physical entities and light sources.

![mix](https://github.com/user-attachments/assets/68a8d21d-ad1e-4ce8-9a12-3f8689447562)

<img width="702" height="788" alt="mix" src="https://github.com/user-attachments/assets/2f4dbae0-80f3-499d-be26-044660d36cfc" />



### Calibration Process
To adapt to varying ambient conditions, the system supports a 10-point calibration. Users sample LDR voltages at 5cm increments up to 50cm; the system then uses linear interpolation to accurately map voltage to distance.

## Work Division: Hardware vs. Software
The design follows a principle where the controller operates actively and independently, while the PC handles the user interface.
* **MCU Side:** Performs core Hard Real-Time computations, pulse width timing via Input Capture, and PWM generation to avoid communication bottlenecks.
* **PC Side:** Handles high-level GUI rendering, binary search algorithms, and linear interpolation for data visualization.

## Finite State Machine (FSM)
The system logic is governed by a Finite State Machine. The MCU starts in a sleep state (LPM) and transitions between modes (Object Detection, Telemeter, Light Detection, etc.) based on RX interrupts received from the PC.



## Project Limitations and Future Improvements
* **Memory Optimization:** Strategic use of limited RAM and Flash memory (16KB) was required to accommodate system logic and data arrays.
* **Sensor Sensitivity:** Improving LDR shielding would help prevent false detections from indirect light and environmental noise.
* **Timer Management:** Dynamic reconfiguration of the two available timers was necessary to handle parallel tasks like PWM output and Input Capture.
