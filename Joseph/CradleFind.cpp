#include "Roboner.h"

bool Roboner::CradleFind() {
	int side = LOW, front = LOW;
	digitalWrite(search, HIGH);
	unsigned long time = millis();
	while ((millis() - time) < cradleDelay) {
		side = digitalRead(SIDE);
		front = digitalRead(FRONT);
		if (front == HIGH || side == HIGH) {
			digitalWrite(search, LOW);
			digitalWrite(video, HIGH);
			baby = found1;
			return true;
		}
		else
			digitalWrite(video, LOW);
	}
	digitalWrite(search, LOW);
	return false;	
}
