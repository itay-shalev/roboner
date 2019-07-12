#include "Roboner.h"

void Roboner::toRoom(int room) {
	switch (room) {
	case 1:
		while (USFR < preTurn) //Driving to main junction
			DriveR();
		stopFor(0.05);
		
		TurnR();	//Turn towards ROOM 1 entrance
		
		while (USF > 28) //Driving toward ROOM 1 entrance
			DriveL();
		stopFor(0.1);
		
		gTankR(55, 90);	//Tank to face ROOM 1
		
		while (USR < 9)	//Making sure not too close to wall
			DriveL();
		alignL(30);
		stopFor(0.1);
		
		if (checkFire() == false)	//Checks for fire
			candleloc = 2;
		break;

	case 11:
		gTankR(55, 90);		//Tank towards starting circle
		alignR(30);
		
		while (USF > 25)	//Driving to starting circle
			DriveR(70);
		stopFor(0.1);
		
		gTankL(55, 90);		//Tank to face main junction
		alignR(30);
		candleloc = 1;
		break;

	case 2:
		gTankR(55, 90);							//Tank to face main junction
		alignR(30);
		
		while (USFR < preTurn) 					//Driving to main junction
			DriveR();
		
		TurnR();								//Turn to face entrance of ROOM 2 & 3
		
		while (USFR < preTurn) 					//Driving to entrance of ROOM 2
			DriveR();
		
		while (USF > 25)						//No wall on the right, Gyro Driving
			gyroFollow(-90, 70);
		stopFor(0.1);
		
		if (USFL > preTurn || USRL > preTurn)  	//Checks ROOM 3's Door Position
			R3_door = oppo;						//ROOM 3's Door is OPPOSITE to ROOM 2's Door
		
		gTankR(55, 90);							//Tank to face ROOM 2
		alignL(30);
		stopFor(0.05);
		
		while (USF > 45 || USRR == MAX_US_DIS) 	//Driving inside ROOM 2
			DriveL();
		stopFor(0.1);
		alignL(30);
		
		
		if (checkFire() == false) {		//*	Checks for fire
			gTankR(50, 45);				//*
			if (checkFire() == false) {	//*
				gTankR(50, 135);		//*
				candleloc = 3;			//*
			}							//*
			else						//*
				candleloc = 22;			//*	^^^^^^^^^^^^^^^
		}
		else
			candleloc = 21;
		break;

	case 3:
		alignR(30);
		
		if (R3_door == oppo) { //ROOM 3's Door is opposite to ROOM 2's Door
			while (USFL < MAX_US_DIS && USRL < MAX_US_DIS)	//Driving while inside ROOM 2
				DriveR(50);
			stopFor(0.1);
		
			TurnL();	//Turn to face main junction
			stopFor(0.1);
		
			gTankR(55, 180);	//180 Tank
		
			while (USFL < preTurn)	//Driving while on corridor
				DriveL();
		
			TurnL();	//Turn inside ROOM 3
		
			while (USRL < preTurn)	//Making sure inside ROOM 3
				DriveR();
			stopFor(0.1);
			alignR(30);
		
			if (checkFire() == false) {		//*	Checks for fire
				gTankL(55, 90);				//*
				if (checkFire() == false) {	//*
					gTankR(55, 90);			//*
					alignR(30);				//*
					candleloc = 42;			//*
				}							//*
				else						//*
					candleloc = 32;			//*
			}								//*
			else							//*
				candleloc = 31;				//* ^^^^^^^^^^^^^^^^
		}
		else {  //ROOM 3's Door is NOT opposite to ROOM 2's Door
			while (USF > 25)       //Exits ROOM 2
				DriveR();
		
			gTankL(55, 90);	//Tank to face main junction
			alignR(30);
		
			while (USFR < preTurn) //Before front hit entrance to ROOM 3
				DriveR(60);
		
			TurnR();	//Turn into ROOM 3
		
			while (USRR < preTurn)	//Making sure inside ROOM 3
				DriveL();
			stopFor(0.1);
			alignL(30);
		
			if (checkFire() == false) {		//*	Checks for fire
				gTankR(55, 90);				//*
				if (checkFire() == false) {	//*
					gTankR(55, 90);			//*
					candleloc = 42;			//*
				}							//*
				else						//*
					candleloc = 32;			//*
			}								//*
			else							//*
				candleloc = 31;				//*	^^^^^^^^^^^^^^^
		}
		break;

	case 41:
		if (Dog != 1) {	//If dog is near starting circle
			while (USR < 72)	//Driving to possibly ROOM 4 entrance
				DriveL();
			stopFor(0.1);
			alignL(30);
		
			gTankR(55, 90);	//Tank to possibly ROOM 4 entrance
		
			if (USF > inR4) {	//Checks if ROOM 4 entrance is there
				R4 = 1;
				if (checkFire() == false)	//*	Checks for fire
					candleloc = 11;
			}
			else {	//ROOM 4 entrance is not there
				R4 = 2;
				candleloc = 11;
			}
			stopFor(0.1);
		}
		else {	//Dog is near starting circle
			while (USFL < preTurn)	//Driving away from main junction
				DriveL();
		
			TurnL();	//Turns left
		
			while (USFL < preTurn)	//Driving till corner
				DriveL();
		
			TurnL();	//Driving to face starting circle
		
			while (USR < 52.5)	//Driving till near ROOM 4 entrance
				DriveR();
			stopFor(0.1);
			alignR(30);
		
			gTankL(55, 90);	//Tank to face ROOM 4 entrance
		
			checkFire();	//*	Checks for fire
		}
		break;

	case 42:
		if (R3_door == oppo) {	//If ROOM 3 door is opposite to ROOM 2 door
			alignR(30);
		
			while (USFL != MAX_US_DIS || USRL != MAX_US_DIS)	//Drive backwards till out of room
				BDriveR(60);
		
			BDriveR(30);
			delay(500);
			stopFor(0.1);
		
			gTankL(55, 90);	//Tank to face the main junction
		
			while (USRR > preTurn)	//Following gyro as there is no wall
				gyroFollow(85, 70);
		
			while (USFR < preTurn)	//Driving to main junction
				DriveR();
		
			TurnR();	//Turn towards ROOM 4
		
			while (USRL > preTurn)	//Making sure in line of ROOM 4
				DriveR(50);
		
			if (USFL > preTurn)	//Checks for ROOM 4
				R4 = 2;
		
			if (R4 == 2) {	//If ROOM 4 is there
				stopFor(0.1);
				gTankL(55, 90);	//Tank to face ROOM 4
				stat = found;
			}
			else
				candleloc = 41;
		}
		else {  //ROOM 3 door is not opposite to ROOM 2 door
			alignR(30);
		
			while (USFR < preTurn)	//Driving while wall is near
				DriveR();
		
			while (USF > 25)	//Driving till close to front wall
				gyroFollow(0, 70);
			stopFor(0.1);
		
			gTankR(55, 90);	//Tank to face main junction
			alignL(30);
		
			while (USFL < preTurn)	//Driving to main junction
				DriveL();
		
			while (USRR < preTurn)	//Making sure the whole robot inside the junction
				gyroFollow(0, 60);
			stopFor(0.1);
		
			gTankR(55, 90);	//Tank to face entrance of ROOM 4
		
			while (USRR > preTurn)	//While no wall driving straight
				gyroFollow(-85, 70);
		
			if (USFL > preTurn)	//Checks for ROOM 4 entrance
				R4 = 2;

			if (R4 == 2) {	//If ROOM 4 entrance is there
				stopFor(0.1);
				gTankL(55, 90);	//Tank to face ROOM 4
				stat = found;
			}
			else
				candleloc = 41;
		}
		break;
	}
}
