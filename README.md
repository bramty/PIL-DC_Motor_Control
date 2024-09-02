# Cascaded PID for Controlling DC Motor's speed and current on Processor-in-the-Loop environment

## Overview

This project designs and implements a cascade PID and PI controller to simultaneously control motor speed and armature current in a DC motor. Initial controller gains are set using the Ziegler-Nichols stability tuning method. By employing the cascade control strategy, it enhances performance by decoupling the speed and current control loops. The controller's effectiveness is tested through Processor-in-the-Loop using serial communication between a DC motor model on Simulink and an Arduino Uno.
![PIL_DCMotor](https://github.com/user-attachments/assets/86e66d77-133a-42e3-856d-e8324e8ccea5)

The details for implementing the serial communication block to interface with Arduino can be found [here](https://github.com/leomariga/Simulink-Arduino-Serial).

## Project Structure
* **Arduino:** containing the code of cascaded speed and current controller and the interface for processor-in-the-loop environment
* **Simulink:** DC motor model implemented on Simulink
* **The Cascaded PID-PI Controller for Speed and Current Regulation of DC Motor with Ziegler-Nichols Stability Limit Tuning Method.pdf:** Full report on the project
