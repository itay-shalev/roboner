#include <Wire.h>
#include <MPU6050.h>

#define BACKWARD 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

//MOTOR CONECTIONS
const int M_FL = 2;
const int M_FR = 4;
const int M_BL = 8;
const int M_BR = 6;
//IR CONECTIONS
const int IR_LR = A2;
const int IR_LL = A3;
const int IR_FR = A0;
const int IR_FL = A1;
//US CONNECTIONS
const int US_FL = 46;
const int US_FR = 48;
const int US_RL = 44;
const int US_RR = 42;
const int US_BL = 40;
const int US_LL = 38;
const int US_LR = 36;
const int US_BR = 34;


//VAREABELS
int speed = 100; 
unsigned long timer = 0;
double yaw = 0;
float timeStep = 0.01;
int us = US_RL;

MPU6050 mpu;

void setup() 
{
  mpu.begin();
  mpu.calibrateGyro();
  pinMode(M_FL, OUTPUT);
  pinMode(M_FR, OUTPUT);
  pinMode(M_BL, OUTPUT);
  pinMode(M_BR, OUTPUT);
  pinMode(IR_LR, INPUT);
  pinMode(IR_LL, INPUT);
  pinMode(IR_FR, INPUT);
  pinMode(IR_FL, INPUT);
  initUS(us);
  
  Serial.begin(9600);
  
  }

void loop() 
{
  //gyroDrive(90, 20);
  Serial.print(readUS(us));
  Serial.print("\n");
  delay(100);

}

void motorControl(int motor, int speed, int dir)
{
  if (dir)
  {
    analogWrite(motor, map(speed, 0, 100, 0, 255));
    digitalWrite(motor + 1, LOW);
  }
  else
  {
    analogWrite(motor + 1, map(speed, 0, 100, 0, 255));
    digitalWrite(motor, LOW);
  }
}

void drive(int speed, int dir)
{
  switch(dir)
  {
    case BACKWARD:
      motorControl(M_FL, speed, FORWARD);
      motorControl(M_FR, speed, BACKWARD);
      motorControl(M_BL, speed, BACKWARD);
      motorControl(M_BR, speed, FORWARD);
      break;
    case FORWARD:
      motorControl(M_FL, speed, BACKWARD);
      motorControl(M_FR, speed, FORWARD);
      motorControl(M_BL, speed, FORWARD);
      motorControl(M_BR, speed, BACKWARD);
      break;
    case LEFT:
      motorControl(M_FL, speed, BACKWARD);
      motorControl(M_FR, speed, BACKWARD);
      motorControl(M_BL, speed, FORWARD);
      motorControl(M_BR, speed, FORWARD);
      break;
    case RIGHT:
      motorControl(M_FL, speed, FORWARD);
      motorControl(M_FR, speed, FORWARD);
      motorControl(M_BL, speed, BACKWARD);
      motorControl(M_BR, speed, BACKWARD);
      break;
  }
}

void stopRobot()
{
  for(int i = M_FL; i <= M_BL + 1; i++)
  {
    digitalWrite(i, LOW);
  }
}

void motorInit(int motor)
{
  pinMode(motor, OUTPUT);
  pinMode(motor + 1, OUTPUT);
}

double getYaw()
{
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  
  yaw += norm.ZAxis * timeStep;
  delay(timeStep * 1000 - (millis() - timer));
  return yaw;
}

void turn(int speed, int dir)
{
  if (dir)
  {
    motorControl(M_FL, speed, FORWARD);
    motorControl(M_FR, speed, FORWARD);
    motorControl(M_BL, speed, FORWARD);
    motorControl(M_BR, speed, FORWARD);
  }
  else
  {
    motorControl(M_FL, speed, BACKWARD);
    motorControl(M_FR, speed, BACKWARD);
    motorControl(M_BL, speed, BACKWARD);
    motorControl(M_BR, speed, BACKWARD);
  }
}

void gyroDrive(int angle, int speed)
{
  // yaw = 0; //Resets the yaw of the robot.
  angle -= speed / 2; //A fix for the degrees based on the offset caused by the speed.
  Serial.print("Degrees: ");
  Serial.print(angle);
  Serial.print("\n");
  if (angle > 0)
  {
    turn(speed, FORWARD);
  }
  else if (angle < 0)
  {
    turn(speed, BACKWARD);
  }
  else
  {
    drive(speed, FORWARD);
  }
  while (abs(getYaw()) <= angle)
  {
    Serial.print(getYaw());
    Serial.print("\n");
  }
  Serial.print(getYaw());
  Serial.print("\n");
  stopRobot();
}

void checkUS()
{
  double currVal = 0;
  for (int i = 34; i <= 46; i += 2)
  {
    currVal = readUS(i);
    Serial.print("US connected in ");
    Serial.print(i);
    Serial.print("  ");
    Serial.print(currVal);
    Serial.print("\n");
    delay(3000);
  }

}

double readUS(int US)
{
  int Echo = US;
  int trig = US + 1;

  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  return pulseIn(Echo, HIGH) * 0.034 / 2;
}

void initUS(int US)
{
  pinMode(US, INPUT);
  pinMode(US + 1, OUTPUT);
}

