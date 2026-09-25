# Secure Industrial Loading Gate

COMP50069 - Hardware, Microcontrollers and Sensors

ESP32-based industrial gate prototype using:
- PIR sensor
- HC-SR04 ultrasonic sensor
- Potentiometer
- Servo motor
- I2C OLED
- Buzzer
- LED
- Push button

## Wokwi Simulation

https://wokwi.com/projects/473945438841677825

## Main Features

- Autonomous gate operation
- Manual Mode
- Latched Safety Halt
- Obstacle detection below 20 cm
- Adjustable 3-10 second hold-open time
- UART commands
- OLED status display
- Hardware-timer-driven safety buzzer

## UART Commands

`m/M` - Manual Mode  
`a/A` - Automatic Mode  
`r/R` - Safety Reset  
`h/H` - Help
