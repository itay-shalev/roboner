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
int PWMVal = 100;
const int FORWARD = 1;
const int BACKWARD = 0;
const int LEFT = 2;
const int RIGHT = 3;

void setup() {
  pinMode(M_FL, OUTPUT);
  pinMode(M_FR, OUTPUT);
  pinMode(M_BL, OUTPUT);
  pinMode(M_BR, OUTPUT);
  pinMode(IR_LR, INPUT);
  pinMode(IR_LL, INPUT);
  pinMode(IR_FR, INPUT);
  pinMode(IR_FL, INPUT);
  Serial.begin(9600);
}

void loop() {
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

void motorControl(int motor, int speed, int dir){
  if (dir)
  {
    analogWrite(motor, speed);
    digitalWrite(motor + 1, LOW);
  }
  else
  {
    analogWrite(motor + 1, speed);
    digitalWrite(motor, LOW);
  }
}

void drive(int speed, int dir)
{
  switch(dir)
  {
    case BACKWARD:
      motorControl(M_FL, PWMVal, FORWARD);
      motorControl(M_FR, PWMVal, BACKWARD);
      motorControl(M_BL, PWMVal, BACKWARD);
      motorControl(M_BR, PWMVal, FORWARD);
      break;
    case FORWARD:
      motorControl(M_FL, PWMVal, BACKWARD);
      motorControl(M_FR, PWMVal, FORWARD);
      motorControl(M_BL, PWMVal, FORWARD);
      motorControl(M_BR, PWMVal, BACKWARD);
      break;
    case LEFT:
      motorControl(M_FL, PWMVal, BACKWARD);
      motorControl(M_FR, PWMVal, BACKWARD);
      motorControl(M_BL, PWMVal, FORWARD);
      motorControl(M_BR, PWMVal, FORWARD);
      break;
    case RIGHT:
      motorControl(M_FL, PWMVal, FORWARD);
      motorControl(M_FR, PWMVal, FORWARD);
      motorControl(M_BL, PWMVal, BACKWARD);
      motorControl(M_BR, PWMVal, BACKWARD);
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
