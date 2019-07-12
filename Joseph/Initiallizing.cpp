#include "Roboner.h"

void Roboner::initial() {
	if (USRR < preTurn || USFR < preTurn) { // Headed North
		alignR(30);
		if (IRLR > 120 || IRLF > 120) { 	// Checks if the dog ON the LEFT
			Dog = 1;
			candleloc = 1;
		}
		else {  							// Turns Left
			gTankL(45, 90);
			resetYaw();
			candleloc = 41;
		}
	}
	else {  								// Headed West
		alignL(30);
		if (IRFL > 100 || IRFR > 100) { 	// Checks if the dog IS in FRONT
			Dog = 1;
			gTankR(45, 90);
			resetYaw();
			candleloc = 1;
		}
		else {
			candleloc = 41;
			resetYaw();
		}
	}
}

void Roboner::initial3() {
	if (USRR < preTurn || USFR < preTurn) { // Headed North
		alignR(30);
		if (IRLR > 120 || IRLF > 120) { 	// Checks if the dog ON the LEFT
			A3Dog = 1;
			cradleRoom = 1;
		}
		else {  							// Turns Left
			gTankL(35, 90);
			resetYaw();
			cradleRoom = 41;
		}
	}
	else {  								// Headed West
		alignL(30);
		if (IRFL > 100 || IRFR > 100) { 	// Checks if the dog IS in FRONT
			A3Dog = 1;
			gTankR(35, 90);
			resetYaw();
			cradleRoom = 1;
		}
		else {
			cradleRoom = 41;
			resetYaw();
		}
	}
}
