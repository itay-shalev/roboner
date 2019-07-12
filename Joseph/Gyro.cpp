#include "Roboner.h"

double Roboner::getYaw() {
	timer = millis();
	Vector norm = mpu.readNormalizeGyro();
	
	yaw = yaw + norm.ZAxis * timeStep;
	delay((timeStep * 1000) - (millis() - timer));
	return yaw;
}

void Roboner::resetYaw() {
	yaw = 0;
}

void Roboner::gyroFollow(int Angle, int Speed) {
	int yawFix = getYaw() - Angle;
	if (yawFix > 0)
		forward(Speed + 15, Speed - 15);
	else if (yawFix < 0)
		forward(Speed - 15, Speed + 15);
	else
		forward(Speed, Speed);
}

void Roboner::TurnR(int Speed) {
	resetYaw();
	while (abs(getYaw()) < 90)
		forward(Speed, Speed / 10);
	while (USRR > preTurn)
		gyroFollow(-90, Speed);
}

void Roboner::TurnL(int Speed) {
	resetYaw();
	while (abs(getYaw()) < 90)
		forward(Speed / 10, Speed);
	while (USRL > preTurn)
		gyroFollow(90, Speed);
}