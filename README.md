manuquad
========

Manuquad Quadcopter Project

The Manuquad uses an arduino mega microcontroller running Multiwii (multiwii.com) version 2.3.  There is a bluetooth 
module on serial port 0 for wireless communication with the Multiwii Configurator.  This allows wireless PID tuning and 
real-time visual sensor displays and calibration capability.  The xbee transceiver is connected to serial port 1 to act
as the RC receiver, receiving Multiwii Serial Protocol (MSP) messages from the xbee on my laptop.  The sensors are on
a chinese pcb wired together on a I2C bus.  These include a 6050 gyro/accel, 5883 magnetometer and bmp085 barometer.  
Currently these are polled at 100Hz by the arduino.  



Processing Code (processing.org):

Uses the procontroll library to read values from a wireless Xbox360 controller and send them as RC values in a
MSP (multiwii serial protocol) message with an Xbee wireless transceiver.  

Throttle - left joystick (up-down)
Roll - right joystick (left-right)
Pitch - right joystick (up-down)
Yaw - analog triggers
