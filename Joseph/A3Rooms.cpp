#include "Roboner.h"

#define R1L 42
#define R1R 15
#define R1F 46
#define R1B 72

#define R2L 72
#define R2R 15
#define R2B 54

#define R31L 65
#define R31R 15
#define R31F 46

#define R32R 15
#define R32B 72

#define R4F 46
#define R4R 15
#define R4L 38

/**	
  *		Takes room and direction and check if val is in range of 12[cm] +-.	
  *
  *		Inputs:
  *			room: 		Number of Room.
  *			direction: 	Which direction to check
  *	 		val: 		The Value to check if in range
  *
  *		Output:
  * 		True if val IS in range, Otherwise returns False.
  * 	
  */
#define US(room, direction, Val) (((R##room##direction + 12) > Val) && ((R##room##direction - 12) < Val))

int Roboner::A3Rooms() {
	int room;
	double b = USR, l = USRL, r = USRR, f = USF;
	
	if (US(1, F, f) && US(1, L, l) && US(1, R, r) && US(1, B, b)) {	//Checks if in ROOM 1
		room = 1;
		home(1);
	}

	else if (US(2, L, l) && US(2, R, r) && US(2, B, b)) {	//Checks if in ROOM 2
		room = 2;
		if (USF > 50)
			R3_door = oppo;
		else
			R3_door = notoppo;
		home(2);
	}

	else if (US(31, F, f) && US(31, L, l) && US(31, R, r)) {	//Checks if in ROOM 3
		room = 3;
		home(3, notoppo);
	}

	else if (US(32, R, r) && US(32, B, b)) {	//Checks if in ROOM 3
		room = 3;
		home(32);
	}

	else if (US(4, F, f) && US(4, L, l) && US(4, R, r)) {	//Checks if in ROOM 4
		room = 4;
		
		while (USF > 25)	//Going forward till wall ahead
			gyroFollow(0, 60);
		stopFor(0.1);
		
		if (IRLF > 120 || IRLR > 120) {      //Checks if dog is on the LEFT
			gTankR(35, 90);
			alignL(30);
		
			if (USF < MAX_US_DIS) {              //Checks if the wall is close (Room 4 door is to the SOUTH)
				while (USF > 25)	//Driving while wall is far
					DriveL();
				
				gTankR(35, 90);	//Tank to the corridor
		
				while (USRR > preTurn)	//Driving while rear is far from wall
					DriveL();
		
				while (USFR < preTurn)	//Driving till front is far from wall
					DriveR();
		
				TurnR();	//Turn towards main junction
		
				while (USFR < preTurn)	//Driving till main junction
					DriveR();
		
				TurnR(60);	//Turn towards starting circle
		
				while (USFR < preTurn)	//Driving to starting circle
					DriveR();
			}
		
			else {                              //Wall is far (Room 4 door is to the NORTH)
				while (USFL < preTurn)	//Driving before main junction
					DriveL(55);
		
				TurnR();	//Turn right to starting circle
		
				while (USFR < preTurn)	//Driving to starting circle
					DriveR();
			}
		}
		else {                              //Dog is not on the LEFT
			gTankR(55, 90);	//Tank right to check where ROOM 4 is
			alignL(30);
			stopFor(0.1);
		
			if (IRFL > 40 && IRFR > 40) {        //If Dog/Wall on the RIGHT (Room 4 door is to the SOUTH)
				gTankL(55, 180);	//180 Tank towards starting circle
				alignR(30);
		
				while (USF > 25 && USFR < preTurn)	//Driving to starting circle
					DriveR();
		
				while (USF > 25)	//Driving to starting circle
					gyroFollow(0, 70);
		
				gTankR(55, 90);
			}
			else {                              //Room 4 door is to the NORTH
				while (USFL < preTurn)	//Driving to main junction
					DriveL();
		
				TurnR();	//Turn to starting
		
				while (USFR < preTurn)	//Driving to staring circle
					DriveR();
			}
		}
	}
	else
		return 0;

	stopFor(0.1);
	
	if (USF > 50) {	//The corridor in Arena A is at the Start
		while (USFL < preTurn)	//Driving till ROOM 2 of Arena B
			DriveL();
	
		while (USFL > preTurn || USRL > preTurn)	//Driving past ROOM 2 of Arena B
			gyroFollow(0, 70);
	
		while (USFL < preTurn)	//Driving till main junction
			DriveL();
	
		TurnL();	//Turn left towards ROOM 1 entrance
	
		gTankR(55, 180);	//180 Tank towards main junction
		alignR(30);
	
		while (USFL < preTurn)	//Driving to main junction
			DriveL();
	
		TurnL();	//Turning to starting circle
	
		while (USF > 20)	//Driving to starting circle
			DriveL();
		stopFor(0.1);
	
		gTankR(55, 180);	//180 Tank to face main junction
		alignR(30);
		startingArena = true;
	}
	else {	//The corridor in Arena A is near ROOM 2 
		gTankL(55, 180);	//180 Tank to face main junction
	
		while (USFR < preTurn)	//Driving to main junction
			DriveR();
	
		TurnR();	//Turn towards ROOM 1
		stopFor(0.1);
	
		gTankR(55, 180);	//180 Tank to face main junction
		alignR(30);
	
		while (USFR < preTurn)	//Driving to main junction
			DriveR();
		stopFor(0.1);
	
		TurnR();	//Turn towards ROOM 2 & 3
		alignR(30);
	
		while (USFR < preTurn)	//Driving while ROOM 2 wall on the right
			DriveR();
	
		while (USFR > preTurn || USRR > preTurn)	//Driving past ROOM 2
			gyroFollow(0, 80);
		stopFor(0.1);
		
		while (USRL < preTurn)	//Driving till starting circle of Arena B
			DriveR();
		stopFor(0.1);
		startingArena = false;
	}

	return room;
}