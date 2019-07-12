#include "Roboner.h"

void Roboner::tankL(int RS) {
	analogWrite(MLB, map(RS, 0, 100, 0, 255));
	analogWrite(MRF, map(RS, 0, 100, 0, 242));
	analogWrite(MLF, 0);
	analogWrite(MRB, 0);
}

void Roboner::tankR(int RS) {
	analogWrite(MLF, map(RS, 0, 100, 0, 255));
	analogWrite(MRB, map(RS, 0, 100, 0, 242));
	analogWrite(MLB, 0);
	analogWrite(MRF, 0);
}

void Roboner::gTankL(int RS, int Angle) {
	resetYaw();
	while (abs(getYaw()) + 7 < Angle)
		tankL(RS);
	stopFor(0.1);
}

void Roboner::gTankR(int RS, int Angle) {
	resetYaw();
	while (abs(getYaw()) + 7 < Angle)
		tankR(RS);
	stopFor(0.1);
}