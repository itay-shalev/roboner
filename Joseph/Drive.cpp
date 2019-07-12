#include "Roboner.h"

void Roboner::forward(int gainL, int gainR) {
	analogWrite(MLB, 0);
	analogWrite(MRB, 0);
	analogWrite(MLF, map(gainL, 0, 100, 0, 255)); //255 max L ENGINE
	analogWrite(MRF, map(gainR, 0, 100, 0, 242)); //242 max R ENGINE
}

void Roboner::backward(int gainL, int gainR) {
	analogWrite(MLF, 0);
	analogWrite(MRF, 0);
	analogWrite(MLB, map(gainL, 0, 100, 0, 255)); //255 max L ENGINE
	analogWrite(MRB, map(gainR, 0, 100, 0, 242)); //242 max R ENGINE
}

void Roboner::stopFor(double sec) {
	analogWrite(MLF, 0);
	analogWrite(MRF, 0);
	analogWrite(MLB, 0);
	analogWrite(MRB, 0);
	delay(sec * 1000);
}

void Roboner::DriveR(int Speed) {
	double fix = USFR - USRR;	/*Difference between front and back.
	* Positive -> front is farther
	* Negative -> back is farther
	*/
	if (USFR > sideWall) {	//Robot is too far from wall
		if (fix > 0)
			forward(Speed + diff + fixWall, Speed - diff);
		else
			forward(Speed - diff + fixWall, Speed + diff);
	}
	else {	//Robot is too close to wall
		if (fix > 0)
			forward(Speed + diff, Speed - diff + fixWall);
		else
			forward(Speed - diff, Speed + diff + fixWall);
	}
}

void Roboner::DriveL(int Speed) {
	double fix = USFL - USRL;	/*Difference between front and back.
	* Positive -> front is farther
	* Negative -> back is farther
	*/	
	if (USFL > sideWall) {	//Robot is too far from wall
		if (fix > 0)
			forward(Speed - diff, Speed + diff + fixWall);
		else
			forward(Speed + diff, Speed - diff + fixWall);
	}
	else {	//Robot is too close to wall
		if (fix > 0)
			forward(Speed - diff + fixWall, Speed + diff);
		else
			forward(Speed + diff + fixWall, Speed - diff);
	}
}

void Roboner::BDriveR(int Speed) {
	double fix = USRR - USFR;	/*Difference between front and back.
	* Positive -> back is farther
	* Negative -> front is farther
	*/
	if (USFR > sideWall) {	//Robot is too far from wall
		if (fix > 0)
			backward(Speed + diff + fixWall, Speed - diff);
		else
			backward(Speed - diff + fixWall, Speed + diff);
	}
	else {	//Robot is too close to wall
		if (fix > 0)
			backward(Speed + diff, Speed - diff + fixWall);
		else
			backward(Speed - diff, Speed + diff + fixWall);
	}
}