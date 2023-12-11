/*
IR Control

This code is for an elegoo smart car V4.0.
The purpose of this code is to have the car be contorlled by a infared remote.
This is done by having:

By retireving and storing any hex value the IR reveiver receives.
Each button on the ir remote has a certain hex value.
If that the received value is = the value of the forward button the car will go forward.
If that the received value is = the value of the Backward button the car will go backward.
If that the received value is = the value of the Right button the car will go Right.
If that the received value is = the value of the Left button the cra will go Left.
 
This is done by using:
-IRremote.h libraries
-analogWrite
-digitalWrite
-if & else if 

Pins for IR receiver:
- Vcc pin    = pin 1 = 5V+
- GND pin    = pin 2 = GND 
- signal pin = pin 1 = pin 9 on arduino

Pins for dc motors:
- AIN1(Direction)             = pin 10 on TB6612FNG = pin 8 on arduino
- PWMA(Speed/amount of power) = pin 9 on TB6612FNG  = pin 5 on arduino
- BIN1(Direction)             = pin 6 on TB6612FNG  = pin 7 on arduino
- PWMB(Speed/amount of power) = pin 7 on TB6612FNG  = pin 6 on arduino
- STBY                        = pin 19 on TB6612FNG = pin 3 on arduino

Go to: https://www.elegoo.com/blogs/arduino-projects/elegoo-smart-robot-car-kit-v4-0-tutorial
For more info on components pin configuration/Data sheets aswell as info on IR control and other functions this car 
can perform.

*/


// Including the ir library to receive ir signals from IR remote.
#include <IRremote.h>

// Define the IR receiver pin
const int IR_PIN = 9;

// Define the IR receiver object
IRrecv irrecv(IR_PIN);

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
  
  digitalWrite(STBY, 1);     // Give power to standby pin.

  // motor A
  pinMode(PWMA, OUTPUT);     // Set PWMA pin as OUTPUT
  analogWrite(PWMA, speed);  // Set the speed of motor A using PWM.
  pinMode(AIN1, OUTPUT);     // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 10);    // Set the direction of motor A (Set it LOW to rotate clockwise)    .
    

  // Motor B
  pinMode(PWMB, OUTPUT);     // Set PWMB pin as OUTPUT
  analogWrite(PWMB, speed);  // Set the speed of motor B using PWM
  pinMode(BIN1, OUTPUT);     // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 1);     // Set the direction of motor B (Set it HIGH to rotate counter clockwise)
}

// Create a function to make the car move Backward with a variable for speed whos value can be defined when called.
void backward(int speed) {
  
  digitalWrite(STBY, 1);       // Give power to standby pin

  // motor 
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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);                           // Begin serial communication at a baud rate of 9600
  irrecv.enableIRIn();                          // Enable the infrared (IR) receiver
}
void loop() {
  // Check if there are any available codes in the buffer
  if (irrecv.decode()) {
    unsigned long receivedCode = irrecv.decodedIRData.decodedRawData;// Retrieve and store the hex value of the received code
    Serial.println(receivedCode);  // Print the received code to the serial monitor

    // Define the codes as variables
    const unsigned long stopCode = 3208707840;        //OK button
    const unsigned long backwardCode = 3927310080;    //BACWARD arrow button
    const unsigned long forwardCode = 3108437760;     //FORWARD arrow button
    const unsigned long leftCode = 3141861120;        //LEFT arrow button
    const unsigned long rightCode = 3158572800;       //RIGHT arrow button

    // Check if the received code matches the stop code.
    if (receivedCode == stopCode) {
      stop();// Stop the motors when the stop code is received.
    }

    // Check if the received code matches the backward code.
    else if (receivedCode == backwardCode) {
      backward(100); // Execute backward function with a speed of 100.
    }

    // Check if the received code matches the forward code.
    else if (receivedCode == forwardCode) {
      forward(100); // Move forward with a speed of 100.
    }

    // Check if the received code matches the left code.
    else if (receivedCode == leftCode) {
      left(100); // Turn left with a speed of 100.
    }

    // Check if the received code matches the right code.
    else if (receivedCode == rightCode) {
      right(100); // Turn right with a speed of 100.
    }

    // Reset the IR receiver for the next signal.
    irrecv.resume();
  }

  
}

