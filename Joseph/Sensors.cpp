#include "Roboner.h"

#define MicroToCenti(x) (((x) * 0.034) / 2)
#define CentiToMicro(x) (((x) * 2) / 0.034)
#define Max_Millis      CentiToMicro(MAX_US_DIS)

double Roboner::usd(int trig, int echo) {
	double distance = 0;
	digitalWrite(echo, LOW);
	delayMicroseconds(2);
	digitalWrite(trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig, LOW);
	distance = MicroToCenti(pulseIn(echo, HIGH, Max_Millis));
	if (distance == 0)
		distance = MAX_US_DIS;
	return distance;
}

double Roboner::usdRaw(int trig, int echo) {
	digitalWrite(echo, LOW);
	delayMicroseconds(2);
	digitalWrite(trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig, LOW);
	return pulseIn(echo, HIGH, Max_Millis);
}

bool Roboner::line() {
	bool first = (white == HIGH) ? false : true;
	bool second = (white == HIGH) ? false : true;
	if (first && second && USFR < 20) {
		return true;
	}
	return false;
}