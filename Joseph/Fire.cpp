#include "Roboner.h"

void Roboner::exR(int Angle) {
	alignL(30);
	
	findCandle(Right);
	
	double candleAngle = -getYaw();
	stopFor(0.1);
	
	fanON;
	while (uvREAD == HIGH);
	fanOFF;
	fireOFF;
	
	gTankR(50, (Angle - candleAngle));
	alignR(30);
	stopFor(0.1);
}

void Roboner::exL(int Angle) {
	alignR(30);
	
	findCandle(Left);
	
	double candleAngle = getYaw();
	stopFor(0.1);
	
	fanON;
	while (uvREAD == HIGH);
	fanOFF;
	fireOFF;
	
	gTankL(50, (Angle - candleAngle));
	alignL(30);
	stopFor(0.1);
}

void Roboner::BexL(int Angle) {
	alignR(30);
	
	findCandle(Left);
	
	double candleAngle = getYaw();
	stopFor(0.1);
	
	fanON;
	while (uvREAD == HIGH);
	fanOFF;
	fireOFF;
	
	gTankR(55, (Angle + candleAngle));
	alignR(30);
	stopFor(0.1);
}

bool Roboner::checkFire() {
	if (uvREAD == HIGH) {
		stat = found;
		return true;
	}
	else
		return false;
}

void Roboner::findCandle(bool Direction) {
	if (pyroL == LOW && pyroR == HIGH) {
		while (pyroL == LOW) {
			tankR(55);
			getYaw();
		}
	}
	else if (pyroL == HIGH && pyroR == LOW) {
		while (pyroR == LOW) {
			tankL(55);
			getYaw();
		}
	}
	else /*(pyroL == LOW && pyroR == LOW)*/ {
		if (Direction == Right) {
			while (pyroL == LOW) {
				tankR(55);
				getYaw();
			}
		}
		else if (Direction == Left) {
			while (pyroR == LOW) {
				tankL(55);
				getYaw();
			}
		}
	}
}
