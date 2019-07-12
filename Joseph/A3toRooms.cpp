#include "Roboner.h"

void Roboner::A3toRooms(int room) {
	switch (room) {
	case 1:
		
		while (USFR < preTurn) //Before Main Junction
			DriveR();
		
		TurnR();
		
		while (USF > 25) //Certain distance from front wall
			DriveL();
		stopFor(0.1);
		
		gTankR(35, 90);	//Tank to face ROOM 1
		
		while (USR < 47)	//To enter ROOM 1
			DriveL();
		alignL(30);
		stopFor(0.1);
		
		if (CradleFind() == false) {	//*	Checks for the cradle
			cradleRoom = 12;			//*
			gTankR(35, 45);				//*
			if(CradleFind() == false)	//*
				cradleRoom = 2;			//* ^^^^^^^^^^^^^^^^^^^^^
		}
		break;

	case 11:
		
		gTankR(35, 90);	//Tank to face the starting circle
		alignR(30);
		
		while (USF > 25)	//To get to the starting circle
			DriveR();
		stopFor(0.1);
		
		gTankL(35, 90);	//Tank to face the main corridor
		cradleRoom = 1;
		break;

	case 2:
		
		gTankR(35, 135);	//Tank to face the exit of ROOM 1
		
		while (USF > 25)	//To get to the exit of ROOM 1
			DriveR();
		
		gTankL(35, 90);	//Tank to face the main junction
		alignR(30);
		
		while (USFR < preTurn) //Before Junction
			DriveR();
		
		TurnR();	//Pivot to the entrance of ROOM 2
		
		while (USFR < preTurn) //Before Entrance to ROOM 2
			DriveR();
		
		while (USF > 25)	//No wall on the right, Gyro Driving
			gyroFollow(-90, 70);
		stopFor(0.1);
		
		if (USFL > preTurn || USRL > preTurn)  //Checks ROOM 3's Door Position
			R3_door = oppo; //ROOM 3's Door is OPPOSITE to ROOM 2's Door
		
		gTankR(35, 90);	//Tank to ROOM 2
		alignL(30);
		stopFor(0.05);

		while (USF > 40 || USRR == MAX_US_DIS) //To get inside ROOM 2
			DriveL(70);
		stopFor(0.1);
		alignL(30);
		
		if (CradleFind() == false) {		//* Checks for the cradle
			gTankR(35, 90);					//*
			if (CradleFind() == false) {	//*
				gTankR(35, 90);				//*
				cradleRoom = 3;				//*
			}								//*
			else							//*
				cradleRoom = 22;			//*
		}									//* ^^^^^^^^^^^^^^^^^^^^^
		else
			cradleRoom = 21;
		break;

	case 3:
		
		alignR(30);
		
		if (R3_door == oppo) { //ROOM 3's Door is opposite to ROOM 2's Door
			while (USFL < MAX_US_DIS && USRL < MAX_US_DIS)	//To get in the corridor
				DriveR(50);
			stopFor(0.1);
			
			TurnL();	//Pivot into the corridor
			stopFor(0.1);
			
			gTankR(35, 180);	//180 Tank to face the other side
			
			while (USFL < preTurn)	//Driving while in the corridor
				DriveL();
			
			TurnL();	//Turn into ROOM 3
			
			while (USF > 50)	//Making sure inside ROOM 3
				DriveR();
			stopFor(0.1);
			alignR(30);
			
			if (CradleFind() == false) {		//*	Checks for the cradle
				gTankL(35, 90);					//*
				if (CradleFind() == false) {	//*
					gTankL(35, 90);				//*
					cradleRoom = 42;			//*
				}								//*
				else							//*
					cradleRoom = 32;			//*
			}									//*
			else								//*
				cradleRoom = 31;				//*	^^^^^^^^^^^^^^^^^^^^^
		}
		else {  //ROOM 3's Door is NOT opposite to ROOM 2's Door
			while (USF > 25)	//Exits ROOM 2
				DriveR();
			
			gTankL(35, 90);		//Tank to face the main junction
			alignR(30);
			
			while (USFR < preTurn) //Before front hit entrance to ROOM 3
				DriveR(60);
			
			TurnR();	//Pivot turn into ROOM 3
			
			while (USF > 50)	//Making sure inside ROOM 3
				DriveL();
			stopFor(0.1);
			alignL(30);
			
			if (CradleFind() == false) {		//*	Checks for the cradle
				gTankR(35, 90);					//*
				if (CradleFind() == false) {	//*
					gTankR(35, 90);				//*
					cradleRoom = 42;			//*
				}								//*
				else							//*
					cradleRoom = 32;			//*
			}									//*
			else								//*
				cradleRoom = 31;				//*	^^^^^^^^^^^^^^^^^^^^^
		}
		break;

	case 41:
		if (A3Dog != 1) {	//Dog is not near the starting circle
			
			while (USR < 70)	//Driving to ROOM 4 possible entrance
				DriveL(65);
			stopFor(0.1);
			alignL(30);
			
			gTankR(35, 90);	//Right tank to possibly ROOM 4 entrance
			
			if (USF > inR4) {				//*	Checks for the cradle
				R4 = 1;						//*
				if (CradleFind() == false)	//*
					cradleRoom = 11;		//*
			}								//*
			else {							//*
				R4 = 2;						//*
				cradleRoom = 11;			//*
			}								//*	^^^^^^^^^^^^^^^^^^^^^
			stopFor(0.1);
		}

		else {	//Dog is near the starting circle
			while (USFL < preTurn)	//Driving till no wall on the left (away from the main junction)
				DriveL();
		
			TurnL();	//Turn left
		
			while (USFL < preTurn)	//Driving till no wall on the left
				DriveL();
		
			TurnL();	//Left turn towards ROOM 4 (starting circle direction)
		
			while (USR < 52.5)	//Driving till entrance to ROOM 4
				DriveR();
			stopFor(0.1);
			alignR(30);
		
			gTankL(35, 90);	//Tank left to ROOM 4 entrance
		
			CradleFind();	//Checks for the cradle
		}
		break;

	case 42:
		if (R3_door == oppo) {	//ROOM 3 door is opposite to ROOM 2 door
			alignL(30);
		
			while (USFR < MAX_US_DIS && USRR < MAX_US_DIS) //Drive till no lon
				DriveL(40);
			stopFor(0.1);
		
			TurnR();
		
			while (USFR < preTurn)
				DriveR();
		
			TurnR();
		
			if (USFL > preTurn)
				R4 = 2;
			if (R4 == 2) {
				stopFor(0.1);
				gTankL(35, 90);
				baby = found1;
			}
			else
				cradleRoom = 41;
		}
		else {  //R3_door == notoppo
			alignR(30);
		
			while (USFR < preTurn)
				DriveR();
		
			while (USF > 25)
				gyroFollow(0, 70);
			stopFor(0.1);
		
			gTankR(35, 90);
			alignL(30);
		
			while (USFL < preTurn)
				DriveL();
		
			while (USRR < preTurn)
				gyroFollow(0, 60);
			stopFor(0.1);
		
			gTankR(35, 90);
		
			while (USRR > preTurn)
				gyroFollow(-85, 70);
		
			if (R4 != 2) {
				if (USFL > preTurn)
					R4 = 2;
			}
			if (R4 == 2) {
				stopFor(0.1);
				gTankL(35, 90);
				baby = found1;
			}
		
			else
				cradleRoom = 41;
		}
		break;
	}
}
