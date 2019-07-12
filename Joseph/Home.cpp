#include "Roboner.h"

void Roboner::home(int room, bool R3) {
	switch (room) {
	
	case 1:
		while (USF > 25)		//Driving to entrance of ROOM 1
			DriveR();
		
		gTankL(55, 90);			//Tank to face main junction
		alignR();

		while (USR < 50)		//Driving while by ROOM 1
			DriveR();
		
		while (USFL < preTurn)	//Driving before main junction
			DriveL();
		
		TurnL();				//Turn to face starting circle
		
		while (USFR < preTurn)	//Driving to starting circle
			DriveL();

		break;

	case 2:
		alignR();
		
		while (USFL < MAX_US_DIS && USRL < MAX_US_DIS)	//Driving while inside ROOM 2
			DriveR();
		stopFor(0.1);
		
		TurnL();										//Turn towards main junction
		
		while (USFL < preTurn)							//Driving to main junction
			DriveL();
		
		TurnL();										//Turn towards ROOM 1
		stopFor(0.1);
		
		gTankL(55, 180);								//180 Tank to face main junction
		
		while (USFL < preTurn)							//Driving to main junction
			DriveL();
		
		TurnL();										//Turn towards starting circle
		
		while (USFR < preTurn)							//Driving to starting circle
			DriveL();

		break;

	case 3:
		if (R3 == oppo) {	//If ROOM 3 door is opposite to ROOM 2 door
			alignR();

			while (USFL != MAX_US_DIS || USRL != MAX_US_DIS)
				BDriveR(70);

										//Driving backwards outside of ROOM 3

			BDriveR(30);
			delay(500);
			stopFor(0.1);
		
			gTankL(55, 90);				//Tank to face main junction

			while (USRL > preTurn)		//As there is no wall, Gyro Driving
				gyroFollow(85, 70);

			while (USFL < preTurn)		//Driving to main junction
				DriveR();

			TurnL();					//Turn towards ROOM 1

			gTankR(55, 180);			//180 Tank to face main junction

			while (USFL < preTurn)		//Driving to main junction
				DriveL();

			TurnL();					//Turn towards starting circle

			while (USFR < preTurn)		//Driving to starting circle
				DriveL();
		}
		
		else {  			//ROOM 3 door is not opposite to ROOM 2 door
			while (USFR < preTurn)		//Driving while there is a wall
				DriveR();

			while (USF > 25)			//Driving to front wall
				gyroFollow(0, 70);
			stopFor(0.1);

			gTankR(55, 90);				//Tank to face main junction
			alignL();

			while (USFL < preTurn)		//Driving to main junction
				DriveL();

			TurnL();					//Turn towards ROOM 1

			gTankR(55, 180);			//180 Tank to face main junction

			while (USFL < preTurn)		//Driving to main junction
				DriveL();

			TurnL();					//Turn towards starting circle

			while (USFR < preTurn)		//Driving to starting circle
				DriveL();
		}

		break;

	case 41:
		alignR();

		while (USF > 25)						//Driving to front wall (outside ROOM 4)
			gyroFollow(0, 70);
		stopFor(0.1);

		if (Dog != 1) {							//Dog doesn't interrupt on way HOME
			gTankL(55, 90);

			while (USF > 25 && USFR < preTurn) 	//Driving towards starting circle
				DriveR();
		
			while (USF > 25)					//Making sure in starting circle
				gyroFollow(0, 70);
		
			gTankR(55, 90);
		}
		else {									//Dog does interrupt on way HOME
			gTankR(55, 90);						//Tank right
			alignL();

			while (USF > 25)					//Driving to the corner
				DriveL(60);
			stopFor(0.1);

			gTankR(55, 90);						//Tank right

			while (USRR > preTurn)				//Driving till the whole robot is near a wall
				DriveL(60);

			while (USFR < preTurn)				// Driving till turn
				DriveR(60);

			TurnR();							//Turn towards main junction

			while (USFR < preTurn)				//Driving to main junction
				DriveR();

			TurnR();							//Turn towards starting circle

			while (USF > 20)					//Driving to starting circle
				DriveL(60);
		}

		break;

	case 42:
		alignR();

		while (USF > 25)
			gyroFollow(0, 70);					//Driving to front wall (outside ROOM 4)

		gTankR(55, 90);							//Tank to face main junction
		alignL();

		while (USFL < preTurn)					//Driving to main junction
			DriveL();

		TurnR();								//Turn towards starting circle

		while (USF > 25)						//Driving to starting circle
			DriveL();

		break;

	case 32:
		while (USRL > preTurn)	//Driving out of ROOM 3
			DriveR();

		while (USRL < preTurn)	//Making sure out of ROOM 3
			DriveR();
		stopFor(0.1);

		gTankL(55, 90);			//Tank left to check dog
		stopFor(0.1);

		if (IRFL < 100 || IRFR < 100) { //No dog near ROOM 4 door
			while (USRL > preTurn)				//Gyro Driving till wall
				gyroFollow(85, 50);

			while (USFR < MAX_US_DIS)			//Driving towards main junction, regardless of ROOM 4 entrance
				DriveL();

			TurnR();							//Turn towards starting circle

			while (USFR < preTurn)				//Driving to starting circle
				DriveL();
		}

		else { 							//There is dog near ROOM 4 door
			gTankR(55, 90);						//Tank right

			while (USR < 95)					//Driving out of ROOM 3
				DriveR(55);

			while (USF > 25)					//Driving till front wall is near
				DriveR(55);
			stopFor(0.1);

			gTankL(55, 90);						//Tank left to face starting circle

			while (USF > 25 && USFR < preTurn)	//Driving to starting circle
				DriveR();

			while (USF > 25)
				gyroFollow(0, 70);				//Making sure in starting circle

			gTankR(55, 90);						//Tank right to face possible exit
		}

		break;
	}

	stat = done;
}