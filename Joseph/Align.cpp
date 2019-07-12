#include "Roboner.h"

void Roboner::alignL(int rate) {
	bool finish = false;
	long milli = millis();
	while (finish == false) {
		int fix = USFLRaw - USRLRaw;
		if (fix > rate)
			tankL(30);
		else if (fix < -rate)
			tankR(30);
		else
			finish = true;
		if (millis() - milli > 750)
			finish = true;
	}
	resetYaw();
}

void Roboner::alignR(int rate) {
	bool finish = false;
	long milli = millis();
	while (finish == false) {
		int fix = USFRRaw - USRRRaw;
		if (fix < -rate)
			tankL(30);
		else if (fix > rate)
			tankR(30);
		else
			finish = true;
		if (millis() - milli > 750)
			finish = true;
	}
	resetYaw();
}