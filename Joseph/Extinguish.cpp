#include "Roboner.h"

void Roboner::extinguish(int room) {
	switch (room) {
	case 1:
		while (USR < 50)
			DriveL();
		stopFor(0.1);
		fireON;
		exR();
		while (USF > 25)
			DriveR();
		break;

	case 21:
		fireON;
		exR();
		candleloc = 2;
		break;

	case 22:
		fireON;
		exR(135);
		candleloc = 2;
		break;

	case 31:
		fireON;
		if (R3_door == oppo)
			BexL();
		else if (R3_door == notoppo)
			exR();
		candleloc = 3;
		break;

	case 32:
		fireON;
		if (R3_door == oppo)
			BexL(90);
		else if (R3_door == notoppo)
			exR(90);
		candleloc = 3;
		break;

	case 41:
		if (Dog == 1) {
			while (USF > 25)
				gyroFollow(85, 50);
		}
		else {
			while (USF > 25)
				gyroFollow(-85, 50);
		}
		stopFor(0.1);
		alignL(30);
		fireON;
		exR();
		break;

	case 42:
		while (USR < inR4)
			gyroFollow(70, 60);
		stopFor(0.1);
		fireON;
		exR();
		break;
	}
	stat = off;
	stopFor(1);
}
