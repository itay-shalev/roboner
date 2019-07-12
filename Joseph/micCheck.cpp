#include "Roboner.h"

void Roboner::micCheck(bool init) {
	SetInitial();
	bool start = false;
	unsigned long lastDebounceTime = 0;
	int lastSound = LOW;
	while (start == false) {
		int reading = digitalRead(mic);
		if (reading != lastSound)
			lastDebounceTime = millis();
		if (reading == HIGH) {
			digitalWrite(soundLED, HIGH);
			if ((millis() - lastDebounceTime) > micDelay)
				start = true;
		}
		else
			digitalWrite(soundLED, LOW);
		lastSound = reading;
	}
	digitalWrite(soundLED, LOW);
	stat = on;
	mpu.calibrateGyro();
	stopFor(0.1);
	if (init)
		initial();
}
