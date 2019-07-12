#include "Roboner.h"

void Roboner::FollowTillLine() {
	while ((abs(USFRRaw - USRRRaw) > 30 && abs(USFLRaw - USRLRaw) > 30) || (USFR > 45 && USRR > 45))
		gTankL(55, 45);
	alignR(30);
	while (true) {
		while (USF > 25) {
			if (USFR > preTurn) {
				while (USFR > preTurn || USRR > preTurn)
					gyroFollow(0, 35);
			}
			else
				DriveR(35);

			if (line())
				goto WhiteLine;
		}
		alignR(30);
		gTankL(55, 90);
		alignR(30);
		if(line() && USFR < 15)
			goto WhiteLine;
	}
WhiteLine:
	stopFor(0.5);
}
