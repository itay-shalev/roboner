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
//VAREABELS
int speed = 100; 
unsigned long timer = 0;
double yaw = 0;
float timeStep = 0.01;

MPU6050 mpu;
mpu.calibrateGyro();

void setup() 
{
  pinMode(M_FL, OUTPUT);
  pinMode(M_FR, OUTPUT);
  pinMode(M_BL, OUTPUT);
  pinMode(M_BR, OUTPUT);
  pinMode(IR_LR, INPUT);
  pinMode(IR_LL, INPUT);
  pinMode(IR_FR, INPUT);
  pinMode(IR_FL, INPUT);
  
  Serial.begin(9600);
	mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
}

void loop() 
{
  Serial.print("LR: ");
  Serial.print(analogRead(IR_LR));
  Serial.print(", LL: ");
  Serial.print(analogRead(IR_LL));
  Serial.print(", FL: ");
  Serial.print(analogRead(IR_FL));
  Serial.print(", FR: ");
  Serial.print(analogRead(IR_FR));
  Serial.print("\n");
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
  delay(timeStep * 1000 - (millis - timer));
  return yaw;
}

void gyroDrive(int angle, int speed)
{
  yaw = 0; //Resets the yaw of the robot.
  int yawFix = 0;
  while (abs(getYaw()) < angle)
  {
    if (angle > 0)
    {
      drive(speed, LEFT);
    }
    else if (angel < 0)
    {
      drive(speed, RIGHT);
    }
    
  }
}