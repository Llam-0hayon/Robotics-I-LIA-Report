/*
Collision Avoidance

This code is for an elegoo smart car V4.0.
The purpsose of this code is to have the car drive around a space and not collide with any object.
This is done by having:

the car check if theres any objects in front of it.
If there IS it will check if there are any objects on its right.
  If NOT it will go forward.
If there IS it it will check if there are any objects on its left.
  If NOT it wil go right.
If there is it will back up and turn 180 degrees at which point it will repeat these instrcutions.
  If NOT it will go left.

To make it follow those instruction we must first find a way for it to detect if theres an obecjet.
It detects objects by having a ultra sonic sensor attached to a servo motor.
We will make it check the value of the ultra sonic sensor by reading calculating the time between snding and receiving a signal then converting from time to cm.
When the distance read is less than the max disnance = to it detecting an object.
If something where to be detected the car would stop .

The servo would be set to 90 degrees.
If the value of the distance read is LESS than the max distnance the servo would move right.
  If the values was GREATER than 20 the car would gor forward. Then start from the begening.
If the value read was LESS than twenty the servo would swivel left.
  If the value was GREATER than max distance the car will go right. Then start from the begening.
If the value is LESS than max distance the car will go backward for 2 seconds than turn 180 degrees. Then start from the begening.
  If it is GREATER than mx distance it will turn left. Then start from the begening.

This mostly requires:
-if else
-Servo.h Library
-digitaWrite 
-analogWrite

The pins for the ultra sonic sensor:
- Vcc pin  =  pin 1 = 5V+
- Trig pin =  pin 2 = pin 13 on arduino
- Echo pin =  pin 3 = pin 12 on arduino
- GND pin  =  pin 4 = GND

Pins for servo motor(P10)
- GND pin    = pin 1 = GND
- Vcc pin    = pin 2 = 5V+
- Signal pin = pin 3 = pin 10 on a arduino

Pins for dc motors
- AIN1(Direction)             = pin 10 on TB6612FNG = pin 8 on arduino
- PWMA(Speed/amount of power) = pin 9 on TB6612FNG  = pin 5 on arduino
- BIN1(Direction)             = pin 6 on TB6612FNG  = pin 7 on arduino
- PWMB(Speed/amount of power) = pin 7 on TB6612FNG  = pin 6 on arduino
- STBY                        = pin 19 on TB6612FNG = pin 3 on arduino

Go to: https://www.elegoo.com/blogs/arduino-projects/elegoo-smart-robot-car-kit-v4-0-tutorial
For more info on components pin configuration/Data sheets aswell as info on collision avoidance and other functions this car 
can perform.
*/

// Including the Servo library for controlling the servo motor.
#include <Servo.h>
// Creating a Servo object named myservo to control the servo motor.
Servo myservo; 

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

// Define the maximum distance for obstacle detection.
int maxDis = 60;

// Defining pin numbers for ultrasonic sensor trigger, echo, and servo signal.
#define trigPin 13                                                                  //Trigger pin
#define echoPin 12                                                                  //Echo pin
#define servoSig  10                                                                // Signal pin 

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

// Create a function to make the servo move to the middle.
void servoMiddle() 
{ 
  stop();                         //Stop the car before moving the servo.
  myservo.write(90);              //Set the servo to 90 degrees.
}                                  .

// Create a function to make the servo move to the right.
void servoRight() {
  stop();                        //Stop the car before moving the servo.
  myservo.write(0);              //Set the servo to 0 degrees.
}

// Create a function to make the servo move to the left.
void servoLeft() {
  stop();                         //Stop the car before moving the servo
  myservo.write(180);             //Set the servo to 180 degrees.
}

// create a function to measure the distance using the ultrasonic sensor.
int distanceRead() {
  digitalWrite(trigPin, LOW);                           // Set trigPin to LOW to ensure a clean pulse.
  delayMicroseconds(2);                                 // Short delay to allow any lingering signals to settle.
  digitalWrite(trigPin, HIGH);                                // Generate a 10-microsecond pulse by setting trigPin to HIGH.
  delayMicroseconds(10);                                      // Generate a 10-microsecond pulse by setting trigPin to HIGH.
  digitalWrite(trigPin, LOW);                           // Reset trigPin to LOW to complete the pulse.
  float duration = pulseIn(echoPin, HIGH);              // Measure the duration of the pulse received on echoPin.
  float distance = (duration / 2) * 0.0343;             // Calculate the distance using the speed of sound (0.0343 cm/microsecond.
  return (int)distance;                                 // Convert the distance to an integer and return the result.
}

void setup() {
   // put your setup code here, to run once:
  myservo.attach(servoSig);                             // Attach the servo to the specified signal pin.
  Serial.begin(9600);                                   // Begin serial communication at a baud rate of 9600.
  pinMode(trigPin, OUTPUT);                             // Set trigPin as an output to trigger the ultrasonic sensor.
  pinMode(echoPin, INPUT);                              // Set echoPin as an input to receive the ultrasonic sensor's echo signal.
}

void loop() {
 // put your main code here, to run repeatedly:
  
  servoMiddle();                                        // Start with the servo in the middle position.
  delay(500);                                           // Wait half a second for the servo to settle.

  int middleDistance = distanceRead();                  // Define middle distance as distanceRead.
  int rightDistance, leftDistance;                      // Define right distance & middle disnatce as intergers.
  Serial.println(middleDistance);                       // Print vale of middledistance to serilamonitor on a new line.

  // Obstacle detected in the middle, turn servo to the right
  if (middleDistance <= maxDis) {
    servoRight();                                                                    // Turn the servo to the right.
    delay(500);                                                                      // Wait half a second for the servo to move.
    rightDistance = distanceRead();                                                  // Define right distance as distanceRead.
    // Obstacle still detected to the right, turn servo to the left
    if (rightDistance <= maxDis) {
      servoLeft();                                                                   // Turn the servo to the left.
      delay(500);                                                                    // Wait half a second for the servo to move.
      leftDistance = distanceRead();
      // Obstacle still detected to the left, move backward for two seconds
      if (leftDistance <= maxDis) {
        backward(SPEED2);
        delay(1000);
        right(SPEED1);
        delay(oneEighty);
        // No obstacle to the left, move the car to the left
      } else {
        left(SPEED2);                                                             // Move the car to the left(by executing forward funtion at SPEED2).
      
      }
    // No obstacle to the right, move the car to the right
    } else {
      right(SPEED2);                                                              // Move the car to the right(by executing forward funtion at SPEED2).
      
    }
  // No obstacle in the middle, move the car forward at SPEED2
  } else {
    forward(SPEED2);                                                              // Move the car forward(by executing forward funtion at SPEED2).
    
  }
}
