#include "Roboner.h"

void Roboner::Arena2()
{
	micCheck();
	while (true)
	{
		if (stat == on)
			toRoom(candleloc);
		else if (stat == found)
			extinguish(candleloc);
		else if (stat == off)
		{
			home(candleloc, R3_door);
			while (USF > 25)
				DriveL();
			stopFor(1);
		}
	}
}

void Roboner::Arena3()
{
	micCheck(false);
	FollowTillLine();
	startingRoom = A3Rooms();
	stopFor(0.1);
	baby = searching;
	initial3();
	while (true) {
		if (baby == searching)
			A3toRooms(cradleRoom);
		else
			stopFor(1);
	}
	
}

void Roboner::WHITELINE(){
	SetInitial();
	CheckSen;
}

void Roboner::SetInitial()
{
	for (int pin = 0; pin < 6; pin++) {
		pinMode(Trig[pin], OUTPUT);
		pinMode(Echo[pin], INPUT);
	}

	//MUFF Pins
	pinMode(mic, INPUT);
	pinMode(fan, OUTPUT);
	pinMode(UV, INPUT);
	pinMode(fireLED, OUTPUT);
	pinMode(soundLED, OUTPUT);
	pinMode(video, OUTPUT);

	pinMode(search, OUTPUT);
	pinMode(FRONT, INPUT);
	pinMode(SIDE, INPUT);

	mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
}

void Roboner::CHECKUS() {
	SetInitial();
	Serial.print("USF = ");
	Serial.print(USF);
	Serial.print(" USR = ");
	Serial.print(USR);
	Serial.print(" USFR = ");
	Serial.print(USFR);
	Serial.print(" USFL = ");
	Serial.print(USFL);
	Serial.print(" USRR = ");
	Serial.print(USRR);
	Serial.print(" USRL = ");
	Serial.println(USRL);
}

void Roboner::CHECKIR() {
	Serial.print("IRFL = ");
	Serial.print(IRFL);
	Serial.print(" IRFR = ");
	Serial.print(IRFL);
	Serial.print(" IRLF = ");
	Serial.print(IRLF);
	Serial.print(" IRFR = ");
	Serial.println(IRLR);
}