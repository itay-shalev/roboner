#ifndef ROBONER_H
#define ROBONER_H

//Includes
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

//True / False
#define oppo    true
#define notoppo false
#define Left    true
#define Right   false

//Constants
#define fixWall 7
#define diff    8

//Speed
#define def_speed 80

//Distance Defines
#define inR3        60.0
#define inR4        47.0
#define sideWall    15.0
#define preTurn     35.0
#define MAX_US_DIS  95.0

//Delays
#define micDelay	200
#define cradleDelay	2500

//Motors Pins Defines
#define MLB 11
#define MLF 5
#define MRB 3
#define MRF 8

//Pins
#define usfr 41,40
#define usfl 47,46
#define usrr 39,38
#define usrl 43,42
#define usf  49,48
#define usr  37,36

//Fire (Pyro, UV), Microphone and Fan Pins
#define pL       	29
#define pR       	28
#define mic      	33
#define UV       	35
#define fan      	34
#define fireLED  	53
#define soundLED 	52
#define	search		16
#define FRONT		15
#define SIDE		14
#define whiteLine	24
#define video		22

//InfraRed Defines
#define IRFR  analogRead(A0) //Pin 0
#define IRFL  analogRead(A1) //Pin 1
#define IRLF  analogRead(A2) //Pin 2
#define IRFLL analogRead(A3) //Pin 3
#define IRLR  analogRead(A4) //Pin 4

//UltraSonic Defines
#define USFR usd(usfr)
#define USFL usd(usfl)
#define USRR usd(usrr)
#define USRL usd(usrl)
#define USF  usd(usf)
#define USR  usd(usr)

#define USFRRaw usdRaw(usfr)
#define USFLRaw usdRaw(usfl)
#define USRRRaw usdRaw(usrr)
#define USRLRaw usdRaw(usrl)
#define USFRaw  usdRaw(usf)
#define USRRaw  usdRaw(usr)

#define fireON   digitalWrite(fireLED, HIGH)
#define fireOFF  digitalWrite(fireLED, LOW)
#define soundON  digitalWrite(soundLED, HIGH)
#define soundOFF digitalWrite(soundLED, LOW)
#define fanON    digitalWrite(fan, HIGH)
#define fanOFF   digitalWrite(fan, LOW)
#define uvREAD   digitalRead(UV)
#define pyroL    digitalRead(pL)
#define pyroR    digitalRead(pR)
#define white    digitalRead(whiteLine)

#define CheckSen (white == HIGH) ? fireOFF : fireON

class Roboner
{

public:

	void Arena2();
	void Arena3();

	void School();

	void CHECKUS();
	void CHECKIR();

	void WHITELINE();

	bool CradleFind();

private:

	//UltraSonic Pins Arrays
	int Trig[6] = { 37, 39, 41, 43, 47, 49 };
	int Echo[6] = { 36, 38, 40, 42, 46, 48 };

	enum candle { on, found, off, done };
	enum cradle { searching, found1, home1, candle1, home2, candle2, home3 };
	//Global Variables
	int candleloc;
	int Dog = 0;
	bool R3_door = notoppo;
	int R4 = 0;
	candle stat;


	//Arena3 Variables
	int startingRoom;
	bool startingArena;
	int cradleRoom;
	int A3R4 = 0;
	int A3Dog = 0;
	bool A3R3_door = notoppo;
	cradle baby;

	Servo A3Servo;

	//Gyro Object and Global Variables: {
	MPU6050 mpu;

	unsigned long timer = 0;
	float timeStep = 0.01;
	double yaw = 0;
	//}

	//Functions Declares: {
	
		///Initialize sensors
		void SetInitial();
		

		///Updates the Yaw value @return Current yaw value
		double getYaw();
		///Resets Yaw value
		void resetYaw();

		/**
		 *	Raw US reading	
		 *
		 *	@param trig The trigger pin
		 *	@param echo The echo pin
		 *
		 *	@return The raw value of US reading (the time it took for the signal).
		 */
		double usdRaw(int, int);
		/**
		 * US reading
		 * 
		 * @param trig The trigger pin
		 * @param echo The echo pin
		 * 
		 * @return The value of US reading in CM.
		 */
		double usd(int, int);
	
		///@return True if saw white line for 2 consecutive readings, otherwise False
		bool line();
	void   forward(int, int);
	void   backward(int, int);
	void   stopFor(double);
	void   gyroFollow(int Angle, int Speed);
	void   tankR(int);
	void   tankL(int);
	
	void   gTankR(int RS, int Angle);
	void   gTankL(int RS, int Angle);
	void   TurnL(int Speed = 80);
	void   TurnR(int Speed = 80);
	void   DriveR(int = def_speed);
	void   DriveL(int = def_speed);
	void   BDriveR(int = def_speed);
	void   alignL(int = 30);
	void   alignR(int = 30);
	void   FollowTillLine();
	void   findCandle(bool Direction);
	void   exL(int = 180);
	void   exR(int = 180);
	void   BexL(int = 0);
	bool   checkFire();
	void   micCheck(bool = true);
	void   initial();
	void   initial3();
	int	   A3Rooms();
	void   toRoom(int);
	void   A3toRooms(int room);
	void   home(int, bool = false);
	void   extinguish(int);
};
#endif