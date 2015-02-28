#include <Wire.h>  //I2C library

// macro to convert 8 bit I2C address to 7 bit.
// See http://www.arduino.cc/en/Reference/Wire for explanation
#define SEVEN_BIT_I2C_ADDRESS(x) ((x) >> 1)


// The following values can be found in the user guide at
// http://www.mindsensors.com/index.php?module=documents&JAS_DocumentManager_op=downloadFile&JAS_File_id=1260

const int I2CLightArrayAddress = SEVEN_BIT_I2C_ADDRESS(0x14);
const int cmdAddress = 0x41;   //address of register for sending commands
const int registerAddress = 0x42;   //address of register to read from lightarray

const char calibrateWhite = 'W';
const byte calibrateBlack = 'B';
const byte sleepSensor = 'D';
const byte wakeSensor = 'P';
const byte configUS = 'A';
const byte configEU = 'E';
const byte configUniversal = 'U';

const int numberofBytes = 8;

// Normally sendCMD works well BUT when it is put to sleep
// it tends to return Nack on the I2C line when trying to 
// wake it up.  Therefore it is best to try it a number of times
// until there is no error.
int sendCMD(byte cmd){
	int err = 1, count = 0;
	flushWire();

	while (err && count++<5){
		Wire.beginTransmission(I2CLightArrayAddress);
		Wire.write(cmdAddress); //Send a command
		Wire.write(cmd);
		err = Wire.endTransmission();
		delay((err>0) ? 100 : 0);  //if err > 0 then delay(100) else no delay
	}
	return err;
}

//Wake up the light array
int WakeLightArray(){
	return sendCMD(wakeSensor);
}

// Sleep the light array
int SleepLightArray(){
	return sendCMD(sleepSensor);
}

//flush out any data in Wire's buffer
void flushWire(){
	while (Wire.available()){
		Wire.read();
	}
}

int GetCalibratedData(byte *data){
	// clean up some stuff
	memset(data, 0, numberofBytes);
	flushWire();

	//Send message to register for data (0x42)
	Wire.beginTransmission(I2CLightArrayAddress);
	Wire.write(registerAddress);

	if (int err = Wire.endTransmission())  {
		return err;
	}
	delay(10);  //pause for station identification (just joking)

	//request & read the 8 bytes
	Wire.requestFrom(I2CLightArrayAddress, numberofBytes);

	int count = 0;
	while (Wire.available() < numberofBytes && count++ < 5){   //wait until it gets the 8 bytes
		delay(100);
	}

	if (Wire.available() < numberofBytes) {//no data came back. check yer wires kiddo.
		return 5;
	}
	for (int i = 0; i<numberofBytes; i++){
		data[i] = Wire.read();
	}
	return 0;
}

void setup(){
	Wire.begin();
	Serial.begin(9600);
	Serial.println("Begin Lightarray");

	// Below is just a SAMPLE on how to run
	// SleepLightArray and WakeLightArray.

	// Unless you want to save power you will probably
	// NEVER need to call SleepLightArray
	while (int err = SleepLightArray()){
		Serial.print("Sleep error ");
		Serial.println(err);
		delay(20);
	}


	delay(5000); // it sleeps 5 seconds.  Don't do this in normal code please
	// I'm just doing this so you see the lights go out for 
	//  seconds

	// According to userguide the light array will go to sleep
	// after 1 min of inactivity.  You can wake it by calling
	// WakeLightArray().   you probably SHOULD call this in setup().
	while (int err = WakeLightArray()){
		Serial.print("Wake error ");
		Serial.println(err);
		delay(20);
	}
}

void loop(){
	byte larray[8] = { 0 };
	// get the calibrated data
	if (int err = GetCalibratedData(larray)){
		Serial.print("Get Data error ");
		Serial.println(err);
	}
	else{//print out data
		for (int i = 0; i<numberofBytes; i++){
			Serial.print(larray[i]);
			Serial.print(" ");
		}
		Serial.println();
	}
	delay(500);

}