/*
************************************************************
** Author: Penjani Mkandawire
** Date: 10/12/2021
** Email: mkandawire15@gmail.com
** Based on code from the XiaoR Geek UNO DS
************************************************************
*/

#include <EEPROM.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
#include <Servo.h>
#include <std_msgs/String.h>

// PWM Motor duration in milliseconds
const int MotorDuration = 100;

// System startup LED indicator. L298 Enable A and B.
int StartupIndicatorLEDPin = A0;
int EnableA = 5;
int EnableB = 6;

// Servo Motor Interfaces: 1 to 4
int Input1 = 8;
int Input2 = 7;
int Input3 = 12;
int Input4 = 13;

// Define motor adjustment flag
int Adjust = 1;

// Motor speed variables
int LeftSpeedHold = 255;
int RightSpeedHold = 255;

// Create servo motors 1 to 4
Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;

// Set initial servo angle values
byte ServoAngle1 = 70;
byte ServoAngle2 = 60;
byte ServoAngle3 = 60;
byte ServoAngle4 = 60;

// These macros define the turning direction of the robot
#define MotorGoForward  { digitalWrite(Input1,LOW);  digitalWrite(Input2,HIGH); digitalWrite(Input3,LOW);  digitalWrite(Input4,HIGH); }
#define MotorGoBackward { digitalWrite(Input1,HIGH); digitalWrite(Input2,LOW);  digitalWrite(Input3,HIGH); digitalWrite(Input4,LOW); }
#define MotorGoRight    { digitalWrite(Input1,HIGH); digitalWrite(Input2,LOW);  digitalWrite(Input3,LOW);  digitalWrite(Input4,HIGH); }
#define MotorGoLeft     { digitalWrite(Input1,LOW);  digitalWrite(Input2,HIGH); digitalWrite(Input3,HIGH); digitalWrite(Input4,LOW); }
#define MotorStop       { digitalWrite(Input1,LOW);  digitalWrite(Input2,LOW);  digitalWrite(Input3,LOW);  digitalWrite(Input4,LOW); }

// ROS Node holder variables
double MotorSpeed = 0;
double RobotXDirection = 0;
double RobotYDirection = 0;

// ROS node variable
ros::NodeHandle nh;

// Function that will be called when receiving command from Raspberry Pi
void HandleSpeedMovementCommand(const geometry_msgs::Twist& movement)
{
  // Extract the X-Axis direction movement from the message
  RobotXDirection = movement.linear.x;

  // Extract the Y-Axis direction movement from the message
  RobotYDirection = movement.linear.y;

  // Extract the speed
  MotorSpeed = movement.linear.z;
}

// Create a subscriber node to the ROS command velocity topic.
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", HandleSpeedMovementCommand);

void setup()
{
  pinMode(StartupIndicatorLEDPin, OUTPUT);
  pinMode(EnableA, OUTPUT);
  pinMode(EnableB, OUTPUT);
  pinMode(Input1,  OUTPUT);
  pinMode(Input2,  OUTPUT);
  pinMode(Input3,  OUTPUT);
  pinMode(Input4,  OUTPUT);

  // Define the servo ports 1 to 4
  Servo1.attach(3);
  Servo2.attach(4);
  Servo3.attach(2);
  Servo4.attach(11);

  // Initialization of servo angle
  InitSteer();

  // Initialize ROS node
  nh.initNode();

  // Set the Baud rate for ROS serial communication
  nh.getHardware()->setBaud(57600);

  // Subscribe to the command velocity topic
  nh.subscribe(cmd_vel);
}

// Function that will be called to adjust speed
void AdjustSpeed(int newSpeed)
{
   analogWrite(EnableB,newSpeed);
   // Save the speed value
   EEPROM.write(0x09,newSpeed);
    
   analogWrite(EnableA,newSpeed);
   // Save the speed value
   EEPROM.write(0x0A,newSpeed);
}

void loop()
{  
  // Going forward
  if (RobotYDirection == 1)
  {
    MotorGoForward;
    delay(MotorDuration);
    MotorStop;
    
    RobotYDirection = 0;
  }

  // Going back
  if (RobotYDirection == -1)
  {
    MotorGoBackward;
    delay(MotorDuration);
    MotorStop;
    
    RobotYDirection = 0;
  }

  // Going right
  if (RobotXDirection == 1)
  {
    MotorGoRight;
    delay(MotorDuration);
    MotorStop;
    
    RobotXDirection = 0;
  }
  
  // Going left
  if (RobotXDirection == -1)
  {
    MotorGoLeft;
    delay(MotorDuration);
    MotorStop;
    
    RobotXDirection = 0;
  }

  // Adjust speed
  if (MotorSpeed >= 55 && MotorSpeed <= 255)
  {
    // When the speed range is 0 ~ 100, it can be converted into PWM. When the speed PWM is lower than 55, the motor will not rotate
    
    LeftSpeedHold = MotorSpeed;
    RightSpeedHold = MotorSpeed;
    
    AdjustSpeed(MotorSpeed);
  
    MotorSpeed = 0;
  }

  // Initial spin
  nh.spinOnce();
}
