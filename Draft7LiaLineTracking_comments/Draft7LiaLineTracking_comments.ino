/*
Line Tracker

This code is for an elegoo smart car V4.0.
The purpsose of this code is to have the car follow a line of black electrical tape using an infared sensor.
This is done by:

Reading the value of each IR sensors.
It wll start by reading the value of the middle IR sensor.
If the middle IS sensor is within the range the car will go forward.
  If Not It will check if the left IR sensor is within the range.
If it IS the car will go left.
  IF its NOT it will check the right IR sensor is within range.
If it is within the range the car will go right.
 If it is NOT the car will execute a funtion where the cra willl sweep from left to right in search of a line to track.

The range ia m refering is the range of analog values the sensor will input if the ir sesnor is over the black line.
This rtange can vary from sesnor to sensor.
It can also vary from room to room depeding onit slighting.
The range can aslo vary depending on the condition of the black line.

This requires the use of:
-if & else if
-analogRead
-digitalWrite
-analogWrite

Pins for IR sensor
- GND     = pin 1 = GND
- Vcc     = pin 2 = 5V+
- Vleft   = pin 3 = pin A2 on arduino
- Vmiddle = pin 4 = pin A1 on arduino
- Vright  = pin 5 = pin A0 on arduino

Pins for dc motors
- AIN1(Direction)             = pin 10 on TB6612FNG = pin 8 on arduino
- PWMA(Speed/amount of power) = pin 9 on TB6612FNG  = pin 5 on arduino
- BIN1(Direction)             = pin 6 on TB6612FNG  = pin 7 on arduino
- PWMB(Speed/amount of power) = pin 7 on TB6612FNG  = pin 6 on arduino
- STBY                        = pin 19 on TB6612FNG = pin 3 on arduino

Go to: https://www.elegoo.com/blogs/arduino-projects/elegoo-smart-robot-car-kit-v4-0-tutorial
For more info on components pin configuration/Data sheets aswell as info on line tracking and other functions this car 
can perform.
*/     

//defining pin numbers for IR sensor
// these values can change depending on brightnees of room DARKER the room LOWER the values it also depends on the sensor
const int pinLeft = A2;
const int pinMiddle = A1;
const int pinRight = A0;        
//defining values for line detection
int lineDetectMin = 250;                      //minimum value read for line to be detected.                
int lineDetectMax = 850;                      //maximum value read for line to be detected.                                     
int detectNotOnGround =910;                   //value read by sesnor when car is no longer on the ground.

// Defining pin numbers for motor control and standby.
int PWMA = 5;                                               // speed/power for motor A.
int AIN1 = 7;                                               // Direction motor B.
int BIN1 = 8;                                               // Direction motor A.
int PWMB = 6;                                               // Speed?power for motor b.
int STBY = 3;                                               // Stand by pin (allows power to motors).

// Defining speed values for different motor speeds.
int MINspeed = 50;                                          // minimum speed before motors struggle.
int SPEED1 = 100;                                           // A speed value.
int SPEED2 = 150;                                           // A speed value.
int SPEED3 = 200;                                           // A speed value.
int MAXspeed = 255;                                         // Define the maximum motor speed.

// Define amount of milliseconds to go 180 degrees when going right
int oneEighty = 648; 

// Below create a function to make the car stop
void stop() {
  
  digitalWrite(STBY, 1);   // Give power to standby pin

  // motor A
  pinMode(PWMA, OUTPUT);   // Set PWMA pin as OUTPUT
  analogWrite(PWMA, 0);    // Set the speed of motor A to zero
  pinMode(AIN1, OUTPUT);   // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 1);   // Set the direction of motor A (Set it HIGH to rotate clockwise)

  // Motor B
  pinMode(PWMB, OUTPUT);   // Set PWMB pin as OUTPUT
  analogWrite(PWMB, 0);    // Set the speed of motor B to zero
  pinMode(BIN1, OUTPUT);   // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 1);   // Set the direction of motor B (Set it HIGH to rotate clockwise)
}

// Create a function to make the car move Forward with a variable for speed whos value can be defined when called.
void forward(int speed) {
  
  digitalWrite(STBY, 1);       // Give power to standby pin

  // motor A
  pinMode(PWMA, OUTPUT);       // Set PWMA pin as OUTPUT
  analogWrite(PWMA, speed);    // Set the speed of motor A using PWM
  pinMode(AIN1, OUTPUT);       // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 1);       // Set the direction of motor A (Set it HIGH to rotate clockwise)

  // Motor B
  pinMode(PWMB, OUTPUT);       // Set PWMB pin as OUTPUT
  analogWrite(PWMB, speed);    // Set the speed of motor B using PWM
  pinMode(BIN1, OUTPUT);       // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 1);       // Set the direction of motor B (Set it HIGH to rotate clockwise)
}

//Create a function to make the car move Left with a variable for speed whos value can be defined when called.
//moving left means spinnig in place clockwise.
//This is done by the left side motors(B) move counter clockwise and having the right side motors(A) move clockwise.
void left(int speed) {
  
  digitalWrite(STBY, 1);     // Give power to standby pin   

  // motor A
  pinMode(PWMA, OUTPUT);     // Set PWMA pin as OUTPUT
  analogWrite(PWMA, speed);  // Set the speed of motor A using PWM
  pinMode(AIN1, OUTPUT);     // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 1);     // Set the direction of motor A (Set it HIGH to rotate clockwise)    
    

  // Motor B
  pinMode(PWMB, OUTPUT);     // Set PWMB pin as OUTPUT
  analogWrite(PWMB, speed);  // Set the speed of motor B using PWM
  pinMode(BIN1, OUTPUT);     // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 0);     // Set the direction of motor B (Set it LOW to rotate counter clockwise)
}

//Create a function to make the car move Right with a variable for speed whos value can be defined when called.
//moving left means spinnig in place counter clockwise.
//This is done by the left side motors(B) move clockwise and having the right side motors(A) move counter clockwise.
void right(int speed) {
  
  digitalWrite(STBY, 1);     // Give power to standby pin   

  // motor A
  pinMode(PWMA, OUTPUT);     // Set PWMA pin as OUTPUT
  analogWrite(PWMA, speed);  // Set the speed of motor A using PWM
  pinMode(AIN1, OUTPUT);     // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 10);    // Set the direction of motor A (Set it LOW to rotate clockwise)    
    

  // Motor B
  pinMode(PWMB, OUTPUT);     // Set PWMB pin as OUTPUT
  analogWrite(PWMB, speed);  // Set the speed of motor B using PWM
  pinMode(BIN1, OUTPUT);     // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 1);     // Set the direction of motor B (Set it HIGH to rotate counter clockwise)
}

// Create a function to make the car move Backward with a variable for speed whos value can be defined when called.
void backward(int speed) {
  
  digitalWrite(STBY, 1);       // Give power to standby pin

  // motor A
  pinMode(PWMA, OUTPUT);       // Set PWMA pin as OUTPUT
  analogWrite(PWMA, speed);    // Set the speed of motor A using PWM
  pinMode(AIN1, OUTPUT);       // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 0);       // Set the direction of motor A (Set it LOW to rotate counter clockwise)

  // Motor B
  pinMode(PWMB, OUTPUT);       // Set PWMB pin as OUTPUT
  analogWrite(PWMB, speed);    // Set the speed of motor B using PWM
  pinMode(BIN1, OUTPUT);       // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 0);       // Set the direction of motor B (Set it LOW to rotate counter clockwise)
}


unsigned long noLineStartTime = 0;  // Variable to store the start time for noLine function

// creating function that moves car to sweep from left to right
void noLine() {
  unsigned long currentTime = millis();

  if (currentTime - noLineStartTime < 200) {
    // 0~200ms(for 200ms): Turn right
    right(100);                                                    // turn the car to the right(by executing right funtion at speed of 100).
  } else if (currentTime - noLineStartTime < 1600) {
    // 200~1600ms(for 1400ms): Turn left
    left(100);                                                    // turn the car to the left(by executing left funtion at speed of 100).
  } else if (currentTime - noLineStartTime < 3000) {
    // 1600~3000ms(for 1400ms): Turn right
    right(100);                                                   // turn the car to the right(by executing right funtion at speed of 100).
  } else if (currentTime - noLineStartTime < 3500) {
    // 3000~3500ms(for 500ms): Stop
    stop();                                 // execute stop funtion to make car stop.
    Serial.print("\t");                     // Print a space to the serial monitor.
    Serial.println("Stop");                 // print exactly whats in "" to Serial monitor on a new line.
  } else {
    // Reset the start time for the next iteration
    noLineStartTime = currentTime;
  }
}


void setup() {
  // put your setup code here, to run once:

Serial.begin(9600);         //Begin serial communication at a baud rate of 9600.
pinMode(pinLeft,INPUT);     //Set left sensor as input
pinMode(pinMiddle, INPUT);  //Set middle sensor as input
pinMode(pinRight, INPUT);   //Set right sensor as input

}


void loop() {
  // put your main code here, to run repeatedly:
  // Read analog sensor values for left, middle, and right sensors.
  int leftPinRead = analogRead(pinLeft);                                                //Read left sensor
  int middlePinRead = analogRead(pinMiddle);                                            //Read middle sensor
  int rightPinRead = analogRead(pinRight);                                              //Read right sensor
  // Check if all sensors indicate 'not on ground' if execute what with{}
  if (leftPinRead > detectNotOnGround && middlePinRead > detectNotOnGround && rightPinRead > detectNotOnGround) {
    stop();                                                                         // execute stop funtion to make car stop.
    Serial.print("\t");                                                             // Print a space to the serial monitor.
    Serial.println("NOT On Ground");                                                // print exactly whats in "" to Serial monitor on a new line.
    // Check if middle sensor detects a line if so do wahts within {}
  } else if (lineDetectMin <= middlePinRead && middlePinRead <= lineDetectMax) {
    forward(100);                                                                   // move the car forward (by executing forward funtion at speed of 100).
    Serial.print("\t");                                                             // Print a space to the serial monitor.
    Serial.println("Go Straight");                                                  // print exactly whats in "" to Serial monitor on a new line.
  } // Check if left sensor detects a line
    else if (lineDetectMin <= leftPinRead && leftPinRead <= lineDetectMax ) {
    left(100);                                                                      // turn the car to the left(by executing left funtion at speed of 100).
    Serial.print("\t");                                                             // Print a space to the serial monitor.
    Serial.println("Turn Left");                                                    // print exactly whats in "" to Serial monitor on a new line.
   // Check if right sensor detects a line
  } else if (lineDetectMin <= rightPinRead && rightPinRead <= lineDetectMax) {
    right(100);                                                                      // turn the car to the right(by executing right funtion at speed of 100).
    Serial.print("\t");                                                              // Print a space to the serial monitor.
    Serial.println("Turn Right");                                                    // print exactly whats in "" to Serial monitor on new line.
  // If no line is detected, perform sweeping motion
  } else {
    noLine();                                                                        //execute no line funtion
    Serial.print("\t");                                                              // Print a space to the serial monitor.
    Serial.println("NO LINE");                                                       // print exactly whats in "" to Serial monitor on a new line.
  }
  // Print analog sensor readings to the serial monitor
  Serial.print("\t");                                                         // Print a space to the serial monitor.
  Serial.print("Analog Reading Left=");                                       // Print a space to the serial monitor & and print exactly whats in "".
  Serial.print(leftPinRead);                                                  // Print value of leftPinRead to serial monitor.
  Serial.print("\t Analog Reading Middle=");                                  // Print a space to the serial monitor & and print exactly whats in "".
  Serial.print(middlePinRead);                                                // Print value of middlePinRead to serial monitor.
  Serial.print("\t Analog Reading Right=");                                   // Print a space to the serial monitor & and print exactly whats in "".
  Serial.println(rightPinRead);                                               // Print value of rightPinRead to serila monitor on a new line.

  delay(1000);// Delay for 1000 milliseconds (1 second) before the next iteration
}