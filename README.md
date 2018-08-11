# HomeTankControl
Arduino code to control water level in tank
How it works?
 When the tank is full, it automatically switches off the water pump.
 It uses a solid state relay designed using Optocoupler (MOC3041) and a triac (BTA 12 or BTA 20){ as the motor needs 7.5 A of current at 230V}

HARDWARE INTERFACES
Input Switch: Determines when to ON or OFF. Connected to pin 2 of arduino, connected to an interrupt routine.

Sensor: IR sensor, activated mechanically. Increases the potential when tank gets full. Connected to A0 pin of arduino.

Seven segment display: 3 digit seven sigment display driven using 74HC595. Connected to arduino pin 8, 7, 6 as Latch, Clock, Data respectively.

Triac: Any triac that can handle 10A or more can be used. Connected to arduino pin 9.

Buzzer: Makes beep only when the motor is off. Connected to arduino pin 12.

Program uploaded to board on 9 - August - 2018
Triac used is BTA 12
