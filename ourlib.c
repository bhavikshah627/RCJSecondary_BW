#ifndef _OURLIB_
#define _OURLIB_
#include "..\..\iFinal_v4\xDrivers\common.h"
#include "common-light-wpointer.h"
#include "..\..\iFinal_v4\Lib\larrayLib.h"

#include "hiMux.c"

#define LM motorA
#define RM motorC

#define hiMux  S1
#define Larray S2
#define clawServo S3
#define lightsense S3
//HI MUx on S1
//Splitter on S4
const tSensors DistEyeL = (tSensors) S4;
const tSensors DistEyeR = (tSensors) S4;

typedef enumWord { noGap, isGap, isSilver } GapType; //not used now
typedef enumWord { clawflat, clawcan, clawopen } ClawState;

ClawState RCXClaw = clawflat;

//#define PROTOTYPE1

#define  DATALOG

#define BatteryLv()   nxtDisplayTextLine(1, "Batt:%d", nAvgBatteryLevel)
#define SkewLeft()   { motor[RM] = 7;  motor[LM] = 4; } //changed 12 to 10 to 9 to 7
#define SkewRight()  { motor[RM] = 4;  motor[LM] = 7; }//changed 5 to 4

#define SYNLM nSyncedMotors = synchAC
#define SYNRM nSyncedMotors = synchCA

#define LEFTTURN(x)  SpecDistTurn(x)
#define RIGHTTURN(x)  SpecDistTurn(-x)

//int HTCS2readColor(tSensors link);
//macros for Sharp distance sensor==========================

#define          LDistID               0x02 //left
//#define					 RDistID							 0x02 //temp
#define					 RDistID							 0x08
#define          DistCommandReg        0x41
#define          DistReadResult        0x42
#define          DistReadDistance      0x42
#define          DistReadVoltage       0x44

#define          DISTON                   0x45
#define          DISTOFF                  0x44

//Define ^^ if DIST-Nx-Sample.c isn't running
///===========================================================
#define getrawEOPD()        (1023 - SensorRaw[EOPD])
#define getcookedEOPD()     sqrt(getrawEOPD() * 10) //<--- bad.
//#define getcookedEOPD()   (sqrt(raw_data * 10))
#define WAITI2C(s)  { while (nI2CStatus[s] == STAT_COMM_PENDING)  wait1Msec(5);}
#define HeadRight CloseClaw()
#define HeadLeft OpenClaw()

#ifdef DATALOG
int LogData[3] = { 0, 0, 0};
#endif


char distCommand(byte distCommand, tSensors DistPort, int sideaddress);

//========================================  Bot Specs  =================================
#ifdef PROTOTYPE1
#define WheelBase 16.5//17.5//15.5//18.0//18.0//14.9//15.0//14.5 //16.0//15.7//15.0//14.7//15.5
#define TireDia 3.25//1//6.7 //7.0 //7.2 //7.0
#define BotLength 20.0
#define GEAR_RATIO 1.0
#endif

#ifndef PROTOTYPE1
#define WheelBase 17.5//18//15.0
#define TireDia 3.4//3.5
#define BotLength
#define GEAR_RATIO 1.0
#endif


const float EncoderPerCM = 360.0/(TireDia * PI); //Calculates how many degrees it needs to turn to move one centimeter
const float EncoderFor360Turn = WheelBase * PI * EncoderPerCM; //Calculates how much degrees it has to turn to turn 360 degrees
const float EncoderFor1Turn = EncoderFor360Turn / 360.0;
//=========================================  For Scanning  =============================
/*const int base = 53;
const int height = 60;
const float hitcorner = atan(height/base);
const float cangle = radiansToDegrees(hitcorner);
const int speed = 1;
//for turning N degrees = EncoderFor360turn  * N / 360*/
//=========================================  For Scanning  =============================
//Global Variables for Distance Sensors------------------------------
int LdistVal = 0;
int RdistVal = 0;

int Raweopd;
int Sonar;

//bool Silver;

int RawLight;
int NormLight;
int FRawLight;
int FNormLight;

int FrontLight;

//scanning globals
int Roomdist;
int SearchDist;
//Line Tracing Light Values
//int Rblack = 17, Rwhite = 37;
//int Lblack = 26, Lwhite = 39;
//int Lavg =  (Lblack + Lwhite )/2;
//int Ravg =  (Rblack + Rwhite )/2;
//int Lsilver = (51 + Lwhite)/2; //46
//int Rsilver = (49 + Rwhite)/2; //44
//int nCompassHeading;

bool isValid(tSensors s);
void SpecDist(float cm);
void initSensors ();
//void OpenClaw();
//void CloseClaw();
int getDistance(tSensors link, int sideaddress);

//void i2c_flush(tSensors s);
//char distCommand(byte distCommand, tSensors DistPort);
const float degPerCM = 360.0*GEAR_RATIO/(PI*TireDia);
int ForwardSpeed= 25;
int BackwardTurnSpeed = -15;
int ForwardTurnSpeed = 20;
//===========================For deciding Scanning========================
typedef enumWord {LongLeft, LongMid, LongRight, ShortLeft, ShortMid, ShortRight, Front} ScanChoice;

ScanChoice choice;

//=============================================================================
#ifdef code2012
void OpenClaw() //warning, clears motor encoder
{
	nMotorEncoder[CLAW] = 0;
	nMotorEncoderTarget[CLAW] = 90;
	motor[CLAW] = 20;
	while(nMotorRunState[CLAW] != runStateIdle)
	{
		wait10Msec(1);
	}
	nMotorEncoder[CLAW] = 0;
}

void CloseClaw()
{
	nMotorEncoder[CLAW] = 0;
	nMotorEncoderTarget[CLAW] = -90;
	motor[CLAW] = -20;
	while(nMotorRunState[CLAW] != runStateIdle)
	{
		wait10Msec(1);
	}
	nMotorEncoder[CLAW] = 0;
}

#endif
//void VicScan()
//{
//}
// cm > 0 : forward
// cm < 0 : backward


void SpecDist(float cm)
{
	float target1;

	target1 = cm * degPerCM;

	nMotorEncoder[LM] = 0;
	nMotorEncoder[RM] = 0;

	nxtDisplayBigTextLine(1, "%d", target1);

	//nMotorEncoderTarget[LM]=target1;
	//SYNRM;
	//nSyncedTurnRatio=100;
	nMotorEncoderTarget[RM] = target1;
	nMotorEncoderTarget[LM] = target1;

	nxtDisplayCenteredBigTextLine(1, "%d", target1);

	if (cm < 0.0)
	{
		motor[RM] = -35;
		motor[LM] = -35;
	}
	else
	{
		motor[RM] = 35;
		motor[LM] = 35;
	}

	while (nMotorRunState[RM] || nMotorRunState[LM] != runStateIdle)
	{
		wait1Msec(10);
	}
}

// degree > 0 : left turn
// degree < 0 : right turn
void SpecDistTurn(int degree)
{
	int target1;
	int target2;
	// target2 = WheelBase * PI * degPerCM * (degree / 360.0);
	target1 = -(EncoderFor1Turn * degree); //negative turn degree for backwards wheel motion
	target2 = EncoderFor1Turn * degree;

	nMotorEncoder[LM] = 0;
	nMotorEncoder[RM] = 0;
	//SYNRM;
	//nSyncedTurnRatio = -100;
	//nxtDisplayBigTextLine(1, "%d", target2);
	//wait10Msec(1000);
	if (degree > 0)
	{
		nMotorEncoderTarget[RM] = target2;
		nMotorEncoderTarget[LM] = target1;
		motor[RM] = 25;
		motor[LM] = -25;
	}
	else
	{
		nMotorEncoderTarget[RM] = target1;
		nMotorEncoderTarget[LM] = target2;
		motor[RM] = -25;  // right turn
		motor[LM] = 25;
	}

	while (nMotorRunState[RM] != runStateIdle)
	{
		wait1Msec(10);
	}
	wait1Msec(50);
	nSyncedMotors = synchNone;

	/*int orig;
	int targett;
	int compval;
	orig = talkToCompass_Mux();
	targett = orig - degree;
	if (targett < 0)
	{
	targett += 360;
	}
	if (targett > 360)
	{
	targett -= 360;
	}
	compval = talkToCompass_Mux();
	while (compval > (targett + 2) || compval < (targett - 2))
	{
	if(degree >= 0)
	{
	motor[RM] = 10;
	motor[LM] = -10;
	}
	else if(degree < 0)
	{
	motor[RM] = -10;
	motor[LM] = 10;
	}
	compval = talkToCompass_Mux();
	}
	motor[RM] = 0;
	motor[LM] = 0;*/
}

int wobble(bool wobbleright)
{
	if (time1(T1) < 2000)
	{
		if (wobbleright == true)
		{
			motor[RM] = 20;
			motor[LM] = 30;
		}
		else if (wobbleright == false)
		{
			motor[RM] = 30;
			motor[LM] = 20;
		}
	}
	else
	{
		ClearTimer(T1);
		if (wobbleright == true)
			wobbleright = false;
		else if (wobbleright == false)
			wobbleright = true;
	}
	return wobbleright;
}




bool isValid(tSensors s)
{
	while (nI2CStatus[s] == STAT_COMM_PENDING)
		wait1Msec(5);   // Wait for I2C bus to be ready

	return true;
}

// author:  by Sean
// get Distance value 10 times and take average
int getDistance(tSensors link, int sideaddress)
{
	int totalDist=0, dist=0;
	const int trials = 5;
	byte replyMsg[8];
	byte distMsg[8];
	distMsg[0] = 2;
	distMsg[1]= sideaddress;
	distMsg[2] = DistReadDistance;

	for(int i = 0; i < trials; i++)
	{
		replyMsg[0] = 0;
		replyMsg[1] = 0;

		WAITI2C(link);
		sendI2CMsg(link, &distMsg[0], 2);      // Send the message, expect 2 bytes back

		WAITI2C(link);
		readI2CReply(link, &replyMsg[0], 2);

		if ( replyMsg[0] == -1 ) { // -1 is reserved to report errors, so
			dist = 0; // change it to zero.
		}
		else{
			dist = (0x00FF & replyMsg[0] );
			dist += ( (0x00FF & replyMsg[1]) <<8 );
		}
		totalDist += dist;
	}

	totalDist = totalDist/trials;
	return totalDist;
}



bool checksilver()
{
	if (NormLight == 100)   // sees silver?
	{
		return false;
	}
	else
	{
		if (RawLight > 620 && RawLight < 1000 && NormLight == 0)   // if it sees sliver but norm light doesnt se

		{					// silver value is between 650 and 1000

			if (RawLight > 620 && RawLight < 1000)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}
}

bool checksilverF()
{
	if (FNormLight == 100)   // sees silver?
	{
		return false;
	}
	else
	{

		if (FRawLight > 475 && FRawLight < 600 && FNormLight == 0)   // if it sees sliver but norm light doesnt se

		{					// silver value is between 650 and 1000

			if (FRawLight > 475 && FRawLight < 600)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}
}

task distanceTask()
//(tSensors DistPort, int sideaddress)
{
	// collect the left side data
	while (true)
	{
		RdistVal = getDistance(DistEyeR, RDistID)/10;
		LdistVal = getDistance(DistEyeL, LDistID)/10;
		talkToEopd_Mux(true, &Raweopd);
		wait10Msec(3);
		Sonar = talkToSonar_Mux();
		wait10Msec(3);
		talkToLight_Mux(&RawLight, &NormLight);
		//wait10Msec(3);
		//talkToLight_Mux2(&FRawLight, &FNormLight);
		FrontLight = SensorValue(S3);
		wait10Msec(5);
	}
}

void initSensors ()
{
	SensorType[DistEyeL] = sensorI2CCustom9V;  // ir distance sensor
	SensorType[DistEyeR] = sensorI2CCustom9V;
	SensorType[hiMux] = sensorI2CCustom;
	SensorType[Larray] = sensorI2CCustom9V;
	SensorType[lightsense] = sensorLightActive;
	SensorType[S4] = sensorTouch;
	StartTask(distanceTask);
	LSsetActive(hiLight);
	LSsetActive(hiLight2);
	//SensorType[hiLight] = sensorLightActive;

	//SensorType[REye] = sensorReflection ;
	//SensorType[LEye] = sensorReflection ;
	//SensorType[EOPD] = sensorAnalogInactive;  // eopd sensor
	//SensorType[NearEye] = sensorAnalogActive; //set epod in long range

	//  SensorType[REye] = sensorLightActive;  nxt light sensor
	// SensorType[LEye] = sensorLightActive;
	nxtDisplayCenteredTextLine(6, "Batt: %d", nAvgBatteryLevel);

	nI2CBytesReady [DistEyeL] = 0;
	nI2CBytesReady [DistEyeR] = 0;
	//i2c_flush (DistEye);
	//distCommand(DISTON, DistEye, true);
	//distCommand(DISTON, DistEye, false);
	distCommand(DISTON, DistEyeL, true);
	distCommand(DISTON, DistEyeR, false);
	nI2CBytesReady[Larray] = 0;
	confirmMUX(Raweopd);
	wakeupLA(Larray);

	wait10Msec(5);
}

char distCommand(byte distCommand, tSensors DistPort, int sideaddress)
{
	byte distMsg[5];
	const byte MsgSize            = 0;
	const byte DistAddress        = 1;
	const byte CommandAddress     = 2;
	const byte Command = 3;

	// Build the I2C message
	distMsg[MsgSize] = 3;
	distMsg[DistAddress] = sideaddress;
	distMsg[CommandAddress] = DistCommandReg ;
	distMsg[Command] = distCommand;

	while (nI2CStatus[DistPort] == STAT_COMM_PENDING)                 // Wait for I2C bus to be ready

	if ( nI2CStatus[DistPort] != NO_ERR ) {                           // probably sensor is missing.
		return (-1);
	}
	sendI2CMsg(DistPort, &distMsg[0], 0);                              // Send the message
	return (1);
}


void i2c_flush(tSensors s)
{
	int n;
	byte dump[8];

	while (nI2CStatus[s] == STAT_COMM_PENDING)
		n = nI2CBytesReady[s];
	while (n > 0) {
		while (nI2CStatus[s] == STAT_COMM_PENDING)
			readI2CReply(s, &dump[0], n);
		while (nI2CStatus[s] == STAT_COMM_PENDING)
			n = nI2CBytesReady[s];
	}
}


void allstop()
{
	motor[LM] = 0;
	motor[RM] = 0;
	//	motor[CLAW] = 0;
	nMotorEncoder[RM] = 0;
	nMotorEncoder[LM] = 0;
	//  nMotorEncoder[CLAW] = 0;
	wait10Msec(5);
}

void motorstop()
{
	motor[LM] = 0;
	motor[RM] = 0;
	nMotorEncoder[RM] = 0;
	nMotorEncoder[LM] = 0;

	wait10Msec(5);
}


#ifdef DATALOG

// ==========================================  Data Logging Start  ==================

task Logging()
{
	TFileHandle fp;     //Initialize Datacollection
	string sfname, str;

	TFileIOResult ret;
	int nsize	= 5000, i=0;

	//CloseAllHandles(ret);
	wait1Msec(100);
	PlaySoundFile("Woops.rso");
	fp = 0;
	sprintf(sfname,"rampls%d.txt", nSysTime/100);
	Delete(sfname, ret);
	OpenWrite(fp, ret, sfname, nsize);

	nxtDisplayTextLine(6, "name=%s", sfname);
	sprintf(str,"Ramp Data Log\r\n");
	WriteString(fp, ret, str);
	nMotorEncoder[RM] = nMotorEncoder[LM] = 0;

	while (++i < 300)
	{
		sprintf(str,"%d/r/n", SensorValue[S3]);
		WriteString(fp, ret, str);
		nxtDisplayTextLine(7, "%d", i);
		wait1Msec(35);
	}
	Close(fp,ret);

	wait1Msec(11);
	while (bSoundActive)
	{
	}
	//Delete(str, ret);
	return;
}

#endif DATALOG

//color sensor ==============================================================================================

//int getColor(long &red, long &green, long &blue)
//{
//	int _color;

//	_color = HTCSreadColor(HTCS2);

//	if (_color < 0) {
//		nxtDisplayTextLine(4, "ERROR!!");
//		while(true)
//		{
//			PlaySound(soundShortBlip);
//			wait10Msec(10);
//		}
//	}

//	HTCSreadRawRGB(HTCS2, red, green, blue);

//	return true;
//}
void AlignSilver()
{
	PlaySound(soundFastUpwardTones);
	//nxtDisplayTextLine(7, "SILVERALIGN");
	//bool silverval;
	//int target1;
	//int target2;
	//int distval;
	//if (choice == LongLeft || choice == ShortLeft)
	//{
	//	target1 = 4;
	//	target2 = 7;
	//	distval = LdistVal;
	//}
	//else if (choice == LongRight || choice == ShortRight)
	//{
	//	target1 = 7;
	//	target2 = 4;
	//	distval = RdistVal;
	//}
	//motor[RM] = motor[LM] = 5;
	//while(true)
	//{
	//	int count = 0; //counts how many tries it takes to see the same value

	//	if (distval > 15)
	//	{
	//		motor[RM] = target2;
	//		motor[LM] = target1;
	//		count = 0;
	//	}
	//	else if (distval < 15)
	//	{
	//		motor[RM] = target1;
	//		motor[LM] = target2;
	//		count = 0;
	//	}
	//	else
	//	{
	//		if (count > 1)
	//		{
	//			break;
	//		}
	//		count += 1;
	//	}
	//	wait10Msec(100);
	//}
	//silverval = checksilver();
	//while(silverval == false)
	//{
	//	motor[LM] = -10;
	//	motor[RM] = -10;
	//}
}

#endif  _OURLIB_
