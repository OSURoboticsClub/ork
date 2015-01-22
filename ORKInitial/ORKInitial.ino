/*

Project: ORK2.0 Kit Control Code
Version: .1
Date: November 2014

Authors:
Nick McComb
Ryan Skeele
Soo-Hyun Yoo


Description:
	Initial code to control ORK Kit.

	This version of the code is _very_ barebones and is intended for people who
	want to figure out what to do on their own.

	There will be future versions that are much more documented and easier to follow.

	PROCEDE AT YOUR OWN RISK.
	
	The most up-to-date version of this code can be found at https://github.com/OSURoboticsClub/ork

*/

#include <Servo.h> //includes the servo library so that you can easily use its functions

//Constants; This is where you should define anything you will use that won't change in your code
const int switchPin = 2;    // switch input
const int motor1Pin = 5;    // H-bridge leg 1 (pin 2, 1A)
const int motor2Pin = 4;    // H-bridge leg 2 (pin 7, 2A)
const int enablePin = 3;    // H-bridge enable pin

const int motor3Pin = 7;    // H-bridge leg 3
const int motor4Pin = 8;    // H-bridge leg 4
const int enable2Pin = 6;

const int shiftRegData = 2;
const int shiftRegClock = 12;

#define trigPin A2
#define echoPin A3

#define MOTOR_MIN 110
#define RIGHT_SCALE (9./11.)   // HACK


//DS - D12
//SH_CP - D13
//ST_CP - D10

Servo sonicServo;  //Create the servo object. This names the servo sonicServo which you will later call in your code.

void setup() { // the word void means that this function won't return anything. The setup function get's called in arduino, at the beginning to define all the pins you will be using.
    // set the switch as an input:
    pinMode(switchPin, INPUT); 
 
    // set all the other pins you're using as outputs:
    pinMode(motor1Pin, OUTPUT);
    pinMode(motor2Pin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    
    pinMode(motor3Pin, OUTPUT);
    pinMode(motor4Pin, OUTPUT);
    pinMode(enable2Pin, OUTPUT);
    
    //Ultrasonic
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    //This is the onboard LED that can be helpful for testing
    pinMode(13, OUTPUT);
    pinMode(13, HIGH);
    
    // This sets the servo to pin 9
    sonicServo.attach(9);
 
    // set enablePin high so that motor can turn on:
    analogWrite(enablePin, 255);
    
    // You will be able to stream the sonar data over the serial terminal (found in tools on the Arduino IDE), by running Serial.println(yourVariable)
    Serial.begin(9600); 
}
    
void loop() { // This is the main loop that will get run. This is where you should put all your magical super awesome avoidance algorithms.

  int x = 1;
  int var;
  //var = readDistance();
  //Initial Motor start PWM is around 30

  float rot = -1;
  float inc = 0.1;
  while (true) {
    driveRobot(1, rot);
    rot += inc;
    if (rot > 0.9 || rot < -0.9) {
      inc *= -1;
    }

    delay(100);
  }
}

void driveRobot(float lin, float rot){
  float left = max(-1, min(1, lin - rot));
  float right = max(-1, min(1, lin + rot));
  driveMotor(1, left);
  driveMotor(2, right);
}

void driveMotor(uint8_t motorSel, float motorSpeed){
  int dir = 1;
  float speed = motorSpeed;
  if (motorSpeed < 0) {
    dir = 0;
    speed *= -1;
  }
  if(motorSel == 1){  //Right Motor
    /* motorDir: forward 1, reverse 0 */
    digitalWrite(motor1Pin, dir);  // set leg 1 of the H-bridge high
    digitalWrite(motor2Pin, 1 - dir);   // set leg 2 of the H-bridge low
    analogWrite(enablePin, MOTOR_MIN + (255-MOTOR_MIN)*speed*RIGHT_SCALE);
  }
  else if (motorSel == 2){  //Left motor
    digitalWrite(motor3Pin, dir);  // set leg 1 of the H-bridge high
    digitalWrite(motor4Pin, 1 - dir);   // set leg 2 of the H-bridge low
    analogWrite(enable2Pin, MOTOR_MIN + (255-MOTOR_MIN)*speed);
  }
}

//This is the distance function. It returns distance from the sonar sensor by sending a pulse waiting then reading the time it takes for the echo. Divide this by 2 (there and back) and by the speed of sound and you get a fairly accurate distance reading.
long readDistance(void){
  long duration, distance; // long is a variable type, meaning the value can be longer (aka more accurate). This line initializes these two variables.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}

