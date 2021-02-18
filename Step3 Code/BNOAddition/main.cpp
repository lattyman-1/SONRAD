//Ben Lattmann C++ file for soundscape with Walabot while threading
#include <SFML/Audio.hpp>
#include <SFML/Audio/Sound.hpp>
#include <cmath>
#include <iostream>
#include <math.h>
#include <limits.h>
//BNO055
//add -lpigpio -lstdc++ to build file
//include imumaths.h, RPi_Sensor.h, and RPi_BNO055 to the program directory
#include <pigpio.h>
#include "RPi_Sensor.h"
#include "RPi_BNO055.cpp"
#include "imumaths.h"
//Walabot
#include "WalabotAPI.h"
#include <stdio.h>
#include <string>
//add -lpthread to the build file
#include <pthread.h>
#include <unistd.h>
#define NUM_THREADS 2

#define TWO_PI 6.28318
//thread stuff
pthread_t tid1, tid2, tid3;
pthread_mutex_t lockWal, lockBno;
double xp,yp,zp;
//Making and defining the sound file
sf::SoundBuffer Buffer;
sf::Int16 rawData[20000];
//BNO Sensor
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();
double xBno, yBno, zBno;

//------------------------------------------------------------
//Let's add in the BNO sensor readings
void* BNO055SensorThread(void* arg)
{
	std::cout<<"test Init"<<std::endl;
	if (gpioInitialise() <0)
	{
		std::cout <<"Initialisation error of the GPIO \n Closing program..."<< std::endl;
		return NULL;
	}
	std::cout<<"good init?"<<std::endl;
	bno._HandleBNO=i2cOpen(bno._i2cChannel,BNO055_ADDRESS_A,0);
	
	std::cout << "Orientation Sensor Raw Data Test" << std::endl;

	/* Initialise the sensor */
	if(!bno.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		std::cout << "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!" << std::endl;
		while(1);
	}

	gpioSleep(PI_TIME_RELATIVE, 0, 1000);

	/* Display the current temperature */
	//int8_t temp = bno.getTemp();
	//std::cout << "Current Temperature: "<< (int)temp << " C" << std::endl;

	bno.setExtCrystalUse(true);
	std::cout << "Calibration status values: 0=uncalibrated, 3=fully calibrated"<<std::endl;

	while (1)
	{
		// Possible vector values can be:
		// - VECTOR_ACCELEROMETER - m/s^2
		// - VECTOR_MAGNETOMETER  - uT
		// - VECTOR_GYROSCOPE     - rad/s
		// - VECTOR_EULER         - degrees
		// - VECTOR_LINEARACCEL   - m/s^2
		// - VECTOR_GRAVITY       - m/s^2
		imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

		/* Display the floating point data */
		//std::cout << "X: " << euler.x() <<  " Y: " << euler.y() << " Z: " << euler.z() << "\t\t";
		
		pthread_mutex_lock(&lockBno);
		xBno = euler.x();
		yBno = euler.y();
		zBno = euler.z();
		pthread_mutex_unlock(&lockBno);
		printf("BNO055 data grabbed\n");
		/*
		// Quaternion data
		imu::Quaternion quat = bno.getQuat();
		Serial.print("qW: ");
		Serial.print(quat.w(), 4);
		Serial.print(" qX: ");
		Serial.print(quat.y(), 4);
		Serial.print(" qY: ");
		Serial.print(quat.x(), 4);
		Serial.print(" qZ: ");
		Serial.print(quat.z(), 4);
		Serial.print("\t\t");
		*/

		/* Display calibration status for each sensor. */
		//uint8_t system, gyro, accel, mag = 0;
		//bno.getCalibration(&system, &gyro, &accel, &mag);
		//std::cout<< "CALIBRATIO: Sys=" << (int)system << " Gyro=" << (int) gyro << " Accel=" << (int) accel << " Mag=" << (int)mag << std::endl;

		gpioSleep(PI_TIME_RELATIVE, 0, 1000*BNO055_SAMPLERATE_DELAY_MS);
	}
	return NULL;
}



void* SoundscapesThread(void* arg)
{
	//SET THE SOUNDSCAPE ARENA
	sf::Sound Sound;
	Sound.setBuffer(Buffer);
	Sound.setLoop(true);

	sf::Listener::setPosition(0.f, 0.f, 0.f);
	sf::Listener::setDirection(1.f, 0.f, 0.f);
	
	
	Sound.setMinDistance(100.f);			//distance when sound is at it's greatest
	Sound.setAttenuation(40.f);			//how fast the sound disappears past the min distance
	Sound.setRelativeToListener(true);	//just treat the stuff as relative to the listener
	
	//Sound is being looped already
	Sound.play();
	
	int xpo,ypo,zpo;
	double xB,yB,zB;
////////////////////////////////////////////////////////////////////////////	
	while(1)
	{
		//retrieving data from sensor
		pthread_mutex_lock(&lockWal);
		//xpo = xp;
		//ypo = yp;
		zpo = zp;
		pthread_mutex_unlock(&lockWal);
		//grab the data from the walabot
		pthread_mutex_lock(&lockBno);
		xB = xBno;
		yB = yBno;
		zB = zBno;
		pthread_mutex_unlock(&lockBno);
		//THIS IS WHERE WE WOULD RETRIEVE DATA FROM THE BNO SENSOR

		if(zB > 0)
		{
			zB = zB - 180;
		}
		else
		{
			zB = zB + 180;
		}
		
		Sound.setPosition(zB*2, 0.f, zpo);
		printf("\n Set Noise Position\n");
		std::cout<<"zB*2: "<<zB*2<<" zpo: "<<zpo<<std::endl;
		usleep(100000);
	}
	return NULL;
	//Should never reach the return statement
}


void* WalabotFunction(void* arg)
{
	//NOW WE INITIALIZE WALABOT STUFF
    SensorTarget* targets;
    int numTargets;
    APP_STATUS appStatus;
    double calibrationProcess; // Percentage of calibration completed, if status is STATUS_CALIBRATING
    // Walabot_GetRawImageSlice - output parameters
    int*    rasterImage;
    int     sizeX;
    int     sizeY;
    double  sliceDepth;
    double  power;
    // ------------------------
    // Initialize configuration
    // ------------------------
    // Walabot_SetArenaR - input parameters		(z-axis)
    double minInCm = 20;
    double maxInCm = 200;
    double resICm = 5;
    // Walabot_SetArenaTheta - input parameters	(X-axis)
    double minIndegrees = -5;
    double maxIndegrees = 5;
    double resIndegrees = 5;
    // Walabot_SetArenaPhi - input parameters	(Y-axis)
    double minPhiInDegrees = 0;
    double maxPhiInDegrees = 40;
    double resPhiInDegrees = 5;
    
    bool mtiMode = true;				//Moving target indicator* When false, will detect any object. True = detects only changing signals
    // Configure Walabot database install location
    Walabot_SetSettingsFolder((char*)"/var/lib/walabot");
    
    //  1) Connect : Establish communication with Walabot.
    Walabot_ConnectAny();
    
	//  2) Configure : Set scan profile and arena
    Walabot_SetProfile(PROF_SENSOR);
	
	// Setup arena - specify it by Cartesian coordinates(ranges and resolution on the x, y, z axes); 
    //  In Sensor mode there is need to specify Spherical coordinates(ranges and resolution along radial distance and Theta and Phi angles).
    Walabot_SetArenaR(minInCm, maxInCm, resICm);
    // Sets polar range and resolution of arena (parameters in degrees).
    Walabot_SetArenaTheta(minIndegrees, maxIndegrees, resIndegrees);
    // Sets azimuth range and resolution of arena.(parameters in degrees).
    Walabot_SetArenaPhi(minPhiInDegrees, maxPhiInDegrees, resPhiInDegrees);
    FILTER_TYPE filterType = mtiMode ?
        FILTER_TYPE_MTI :       //Moving Target Identification: standard dynamic-imaging filter
        FILTER_TYPE_NONE;
    Walabot_SetDynamicImageFilter(filterType);
    
	//  3) Start: Start the system in preparation for scanning.
    Walabot_Start();
    
	//  4) Start Calibration - only if MTI mode is not set - (there is no sense executing calibration when MTI is active)
    if (!mtiMode) 
    {
        // calibrates scanning to ignore or reduce the signals
        Walabot_StartCalibration();
    }
    bool recording = true;
	//THIS IS ALL FOR THE WALABOT INITIALIZATION
	
	while(1)
	{
		Walabot_GetStatus(&appStatus, &calibrationProcess);
		//  5) Trigger: Scan(sense) according to profile and record signals to be available for processing and retrieval.
		Walabot_Trigger();
		//  6)  Get action : retrieve the last completed triggered recording 
		Walabot_GetSensorTargets(&targets, &numTargets);
		Walabot_GetRawImageSlice(&rasterImage, &sizeX, &sizeY, &sliceDepth, &power);
		//PrintSensorTargets(targets, numTargets);
		int targetIdx;
		double xpos, zpos;
		if (numTargets > 0)
		{
			xpos = targets[0].xPosCm;
			zpos = targets[0].zPosCm;
		}
		else
		{
			xpos = 0;
			zpos = 1000;
			printf("No target detected\n");
		}
		
		//Mutex Lock stuff
		pthread_mutex_lock(&lockWal);
		//xp = xpos;
		//yp = ypos;
		zp = zpos;
		pthread_mutex_unlock(&lockWal);

		printf("\n Walabot Data Grabbed\n");
		usleep(1000);
		//unlock the walabot data
	}
	return NULL;
	//should never reach return statement
}



void SineGenerate(double SAMPLES, double SAMPLE_RATE, double AMPLITUDE)
{
	//start with generating SINE wave
	//const unsigned SAMPLES = 20000;
	//const unsigned SAMPLE_RATE = 80000;
	//const unsigned AMPLITUDE = 30000;
	
	const double increment = 440./40000; 							//figure this stuff out???
	double x = 0;
	for (unsigned i = 0; i < SAMPLES/2; i++) {						//generate the tone
		rawData[i] = AMPLITUDE * sin(x*TWO_PI);
		x += increment;
	}
	for (unsigned i = SAMPLES/2 + 1; i < SAMPLES; i++)				//adds the pause in sound, blinks the sound
	{
		rawData[i] = 0;
	}
	//SINE WAVE IS GENERATED NOW
	printf("\n Initial Sine Generated\n");
	return;
}	
	
	

int main()
{
	//create and load sine wave
	double SAMPLES = 20000;
	double SAMPLE_RATE = 80000;
	double AMPLITUDE = 30000;
	
	SineGenerate(SAMPLES,SAMPLE_RATE,AMPLITUDE);			//Call function that generates sine
	
	//sf::SoundBuffer Buffer;										//made into global variable
	if (!Buffer.loadFromSamples(rawData, SAMPLES, 1, SAMPLE_RATE))	//loads the sine wave for playing
	{
		printf("\n Loading Failed!\n");
		return 1;
	}
	else
	{
		printf("\n Successfully Loaded Sine Wave!\n");				//Should be successfully loaded
	}
	//END SINE WAVE STUFF

////////////////////////////////////////////////////////////////////////////

	//CREATE THREAD STUFF
	//step 1: define the initial variables
	int error;
	if (pthread_mutex_init(&lockWal, NULL) != 0) { 
        printf("\n Walabot mutex init has failed\n"); 
        return 1; 
    }
	if (pthread_mutex_init(&lockBno, NULL) != 0) { 
        printf("\n BNO mutex init has failed\n"); 
        return 1; 
    } 
	//step 2: create thread and raise error if it fails
	error = pthread_create(&tid1, NULL, &WalabotFunction, NULL);
	if (error != 0)
	{
		printf("\nWalabot Thread can't be created\n");
		return 1;
	}
	printf("\n Walabot Thread Created\n");
	//WALABOT THREAD CREATED
	error = pthread_create(&tid3, NULL, &BNO055SensorThread, NULL);
	if (error != 0)
	{
		printf("\nBNO055 Thread can't be created\n");
		return 1;
	}
	printf("\n BNO055 Thread Created\n");
	//WALABOT THREAD CREATED
	error = pthread_create(&tid2, NULL, &SoundscapesThread, NULL);
	if (error != 0)
	{
		printf("\nSinescape Thread can't be created\n");
		return 1;
	}
	printf("\n Sinescape Thread Created\n");
	//SOUNDSCAPE THREAD CREATED
////////////////////////////////////////////////////////////////////////////
	
		//printf("General Looping\n");
		//empty loop to prevent from returning
	
	//will need a thread for the BNO sensor-> TBD
	//write on it's own, can make a function and call it as a new thread
	
	//pthread_join(tid1, NULL);
	//pthread_join(tid2, NULL);
	
	while(1)
	{
		
	}

	pthread_mutex_destroy(&lockWal);
	pthread_mutex_destroy(&lockBno);
	
	return 0;
}
