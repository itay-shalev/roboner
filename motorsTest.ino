
/* The program assumes that the motor's pins are connected next to eachother, meaning that for example
 * the pins of the FrontLeft motor are 5 and 6, or 2 and 3...
 */

//PLACEHOLDER VALUES
int FLPin = 0;
int FRPin = 0;
int BLPin = 0;
int BRPin = 0;
int PWMSpeed = 0;


void setup(){
    pinMode(FLPin, OUTPUT);
    pinMode(FLPin + 1, OUTPUT);
    pinMode(FRPin, OUTPUT);
    pinMode(FRPin + 1, OUTPUT);
    pinMode(BLPin, OUTPUT);
    pinMode(BLPin + 1, OUTPUT);
    pinMode(BRPin, OUTPUT);
    pinMode(BRPin + 1, OUTPUT);
}

void loop(){
    motorControl(FLPin, PWMSpeed);
    motorControl(FRPin, PWMSpeed);
    motorControl(BLPin, PWMSpeed);
    motorControl(BRPin, PWMSpeed);
}

void motorControl(int motorPin, int speed){
    analogWrite(motorPin, speed);
    analogWrite(motorPin + 1, LOW);
}