//HOME WATER PUMP CONTROLLER
/*
 * USES A PRESSURE SENSOR TO SENSE THE AMOUNT OF WATER IN THE TANK
 * TO TURN OFF MOTOR AN INFRARED SENSOR IS USED
 * MOTOR IS DRIVEN USING AN OPTOCOUPLER -- WHICH DRIVES A TRIAC (BTA 12)
 * ONCE TANK IS FULL A BUZZER WILL BE ACTIVEATED TO TURN OFF THE MAINS
 * 
 * TURN ON MAINS --> MEASURE MAINS VOLTAGE (OPTIONAL)
 *               --> MEASURE WATER LEVEL IN TANK --> IF WATER LEVEL IS MORE THAN 80% GIVE A WARNING BY BLINKING OR BEEPING
 * MAINS ON && TANK NOT FULL && PUSH BUTTON PRESSED FIRST TIME --> ACTIVATE OPTOCOUPLER THROUGH PIN NUMBER 13 OF ARDUINO
 *                                                             --> DISPLAY ON LCD "ON" AND START COUNTING TIME
 *                                                             --> TIME AND "ON" WILL BE DISPLAYED ALTERNATELY
 *                                                             --> ANOTHER DISPLAY TO DISPLAY THE PERCENTAGE OF WATER
 * TANK FULL --> DEACTIVATE OPTOCOUPLER
 *           --> ACTIVATE BUZZER TO BEEP
 *           --> CHANGE DISPLAY TO "FULL"
 */
 // CONSTANTS
 #define FULL HIGH
 #define NOT_FULL LOW
 #define ON HIGH
 #define OFF LOW
 
 const uint8_t tankLevelSensor = A0;  // Setting the analog pin A0 as the input pin
 const uint8_t motorDrivePin = 9;  // Pin number 8 will drive the motor
//  const uint8_t motorSwitch = 2;  // Pin 7 will decide to ON or OFF the motor
 const uint8_t buzzerPin = 12;  // Pin 12 is connected to the gate of a triac BT
 const uint8_t latchPin = 7;
 const uint8_t clockPin = 8;
 const uint8_t dataPin = 6;
 const int TANK_LEVEL_THRESHOLD = 560;  // Threshold value for the level sensor
 // VARIABLES THAT CHANGE IN RUNTIME
//  volatile bool motorSwitchState = LOW;
//  volatile unsigned long lastDebounceTime = 0;
 bool motorState = LOW;
 bool tankIsFull = NOT_FULL;

 // Strings for the display
 String motorOn = "ON";
 String motorOff = "OFF";
 char *onPtr = &motorOn[0];
 char *offPtr = &motorOff[0];
 int count = 0;

 void setup()
 {
  Serial.begin(9600);
  // SETTING THE INPUT PINS
  //pinMode(tankLevelSensor, INPUT);
  
  // OUTPUT PIN MODES ARE DEFINED HERE
  // Input pullup is enabled as the interrupt is activated on input low
  // pinMode(motorSwitch, INPUT_PULLUP);
  pinMode(motorDrivePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);  // Mute the buzzer while setting up everything
  // Interrupt is initiated, motor_control function is called when the interrupt is
  // activated, and is connected to pin 2, activated when the pin goes LOW
  // attachInterrupt(digitalPinToInterrupt(motorSwitch), motor_control, LOW);
  
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  delay(200);
  
  blank_display();
  delay(200);
  
  dashed_line();
  delay(200);
  count = 0;
 }
 
 void loop()
 {
  tankIsFull = checkTankState(tankLevelSensor, TANK_LEVEL_THRESHOLD);
  if(tankIsFull){
    count = 1;
  }

  if(tankIsFull){
    motorState = OFF;
  }else{
    motorState = ON;
  }
  
  if((motorState == ON)&&(count == 0)){
    digitalWrite(motorDrivePin, ON);  // Switch on the motor
    Serial.println("Motor is ON.");
    motor_ON(); // Displays On in seven segment display
    digitalWrite(buzzerPin, LOW);
    delay(100);
  }
  else{
    digitalWrite(motorDrivePin, OFF); // Switch off the motor
    Serial.println("Motor is OFF.");
    motor_OFF();  // Displays OFF in seven segment display
    beep(buzzerPin);    
    blank_display();
    if(count != 0){
      ful();  // Displays FUL
      beep(buzzerPin);
    }
  }

  Serial.println(analogRead(tankLevelSensor));
 }

 /* ############## FUNCTIONS ###################### */
 /* Interrupt service routine
  */
// void motor_control()
// {
//   /* This function will be called when the interrupt is triggered
//    * Acts as a debounce swtich */
//   unsigned long presentTime = 0;
//   presentTime = millis();
//   if((presentTime - lastDebounceTime)>100){
//     motorSwitchState = !motorSwitchState;
//   }
//   lastDebounceTime = presentTime;
// }

/* Function that checks tank is full or not
 *  input arguments are the analog sensor pin and threshold sensor value
 */
bool checkTankState(uint8_t sensor_pin, int threshold)
{
  return (analogRead(sensor_pin) > threshold);
}

/* Display ON in seven segment display*/
 void motor_ON()
 {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, 256);
  shiftOut(dataPin, clockPin, MSBFIRST, 63);
  shiftOut(dataPin, clockPin, MSBFIRST, 84);
  digitalWrite(latchPin, HIGH);
  delay(500);
 }

/* Display OFF */
void motor_OFF()
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, 63);
  shiftOut(dataPin, clockPin, MSBFIRST, 113);
  shiftOut(dataPin, clockPin, MSBFIRST, 113);
  digitalWrite(latchPin, HIGH);
}

/* Blank display*/
void blank_display()
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, 256);
  shiftOut(dataPin, clockPin, MSBFIRST, 256);
  shiftOut(dataPin, clockPin, MSBFIRST, 256);
  digitalWrite(latchPin, HIGH);
}

/* Dashed line display*/
void dashed_line()
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, 8);
  shiftOut(dataPin, clockPin, MSBFIRST, 8);
  shiftOut(dataPin, clockPin, MSBFIRST, 8);
  digitalWrite(latchPin, HIGH);
}

/* Display FUL */
void ful()
{
  int ful[4] = {113, 62, 56, 56};
  // 0 1 2
  // 1 2 3
  for(int i=0; i<2; i++)
  {
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, ful[i]);
    shiftOut(dataPin, clockPin, MSBFIRST, ful[i+1]);
    shiftOut(dataPin, clockPin, MSBFIRST, ful[i+2]);
    digitalWrite(latchPin, HIGH);
    if(i == 0){
      delay(500);    
    }
    else{
      delay(200);
    }
  }
}

/* Make beep */
void beep(uint8_t pin)
{
  digitalWrite(pin, HIGH); 
  delay(200);
  digitalWrite(pin, LOW); 
  delay(400);
}
