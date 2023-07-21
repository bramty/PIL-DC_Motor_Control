# PIL-DC_Motor_Control
Cascaded PID for Controlling DC Motor's speed and current on Processor-in-the-Loop environment

In this work, we create a model of the DC motor using Simulink and implement a cascaded PID control algorithm on Arduino Uno to regulate the speed and current of a DC motor. To establish communication between the DC motor model and the control algorithm, we employ serial communication on both Arduino and Simulink. The details for implementing the serial communication block to interface with Arduino can be found [here](https://github.com/leomariga/Simulink-Arduino-Serial).

For the complete explanation, please refer to "The Cascaded PID-PI Controller for Speed and Current Regulation of DC Motor with Ziegler-Nichols Stability Limit Tuning Method.pdf"
