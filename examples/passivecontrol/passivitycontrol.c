#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "drdc.h"
#include <iostream>
#include <string>

#define MIN(x,y) (x)<(y)?(x):(y)
#define MAX(x,y) (x)>(y)?(x):(y)

#define DEFAULT_K 12.0
#define N 3
#define DELAYCONST 10
#define DELAYCONSTMAX 2000
#define DELAYSTEP 10
#define CYCLETIME 0.127e-3
#define ALPHA 0.05 // 0.08
#define ENC 20300 // encoder reading corresponding to origin

#define PI 3.1415926
#define FREQN 1.0e2
#define zeta 0.707

double mp[N] = { 0.0 }; // end-effector position
double sp[N] = { 0.0 };

double mj[N] = { 0.0 }; // joint angle
double sj[N] = { 0.0 };

double mv[N] = { 0.0 }; // angular velocity
double sv[N] = { 0.0 };

double mvBackv[2][N] = { 0.0 }; // last step angular velocity
double svBackv[2][N] = { 0.0 };

double mjBack[DELAYCONSTMAX][N] = { 0.0 }; // list of backstep joint angle to simulate delay
double sjBack[DELAYCONSTMAX][N] = { 0.0 };

double mvBack[DELAYCONSTMAX][N] = { 0.0 }; //list of backstep angular velocity to simulate delay
double svBack[DELAYCONSTMAX][N] = { 0.0 };

double mjDelay[N];
double mvDelay[N];

double sjDelay[N];
double svDelay[N];

double mt[N]; // output torque
double st[N];

double Kp = DEFAULT_K;
double Kv = Kp / 400;
double Kd;

double omegan = 2 * PI * FREQN;

int delayConst = DELAYCONST;

int    done = 0;

double velocityCalculator(double vBack1, double vBack2, double vEst0, double vEst1, double vEst2, double time) {
	double K = omegan / tan(omegan * time / 2);	
	double v;
	double a0 = K * K + 4 * K * zeta * omegan + omegan * omegan;
	double a1 = 2 * omegan * omegan - 2 * K * K;
	double a2 = K * K - 4 * K * zeta * omegan + omegan * omegan;
	double b0 = omegan * omegan;
	double b1 = 2 * omegan * omegan;
	double b2 = omegan * omegan;
	//v = vEst0 * ALPHA + (1 - ALPHA) * vBack1;
	v = (b0 * vEst0 + b1 * vEst1 + b2 * vEst2 - a1 * vBack1 - a2 * vBack2) / a0;
	return v;
}

// master's haptic loop
void *masterThread(void *arg)
{
	int master = *((int*)arg);

	double mq[N]; // output torque for gravity compensation
	double mRefTime = dhdGetTime();
	double mTempTime;
	double mCycleTime;
	int mDelayIndex = 0;

	double mvEst[N][N] = { 0.0 };

	while (!done) {

		// get master's position and velocity
		dhdGetDeltaJointAngles(&mj[0], &mj[1], &mj[2], master);
		dhdDeltaGravityJointTorques(mj[0], mj[1], mj[2], &mq[0], &mq[1], &mq[2], master);

		mTempTime = dhdGetTime();
		mCycleTime = mTempTime - mRefTime;
		mRefTime = mTempTime;

		// calculate master's velocity
		for (int i = 0; i < N; i++) {
			if (mDelayIndex == 0)	mvEst[0][i] = (mj[i] - mjBack[DELAYCONSTMAX - 1][i]) / mCycleTime;
			else	mvEst[0][i] = (mj[i] - mjBack[mDelayIndex - 1][i]) / mCycleTime;
			mv[i] = velocityCalculator(mvBackv[0][i], mvBackv[1][i], mvEst[0][i], mvEst[1][i], mvEst[2][i], mCycleTime);
			mvBackv[1][i] = mvBackv[0][i];
			mvBackv[0][i] = mv[i];
			mvEst[2][i] = mvEst[1][i];
			mvEst[1][i] = mvEst[0][i];
		}

		// set backward position and velocity
		for (int i = 0; i < N; i++) {
			if (mDelayIndex - delayConst < 0) {
				mjDelay[i] = mjBack[mDelayIndex + DELAYCONSTMAX - delayConst][i];
				mvDelay[i] = mvBack[mDelayIndex + DELAYCONSTMAX - delayConst][i];
			}
			else {
				mjDelay[i] = mjBack[mDelayIndex - delayConst][i];
				mvDelay[i] = mvBack[mDelayIndex - delayConst][i];
			}
			mjBack[mDelayIndex][i] = mj[i];
			mvBack[mDelayIndex][i] = mv[i];
		}

		// calculate master's joint torque 
		mt[0] = Kv * (svDelay[0] - mv[0]) - Kd * mv[0] + Kp * (sjDelay[0] - mj[0]) + mq[0];
		mt[1] = Kv * (svDelay[2] - mv[1]) - Kd * mv[1] + Kp * (sjDelay[2] - mj[1]) + mq[1];
		mt[2] = Kv * (svDelay[1] - mv[2]) - Kd * mv[2] + Kp * (sjDelay[1] - mj[2]) + mq[2];

		dhdSetDeltaJointTorques(mt[0], mt[1], mt[2], master);
		dhdGetPosition(&mp[0], &mp[1], &mp[2], master);

		if (mDelayIndex == DELAYCONSTMAX - 1) mDelayIndex = 0;
		else mDelayIndex++;

	}

	// close the connection
	//drdClose(master);
	return NULL;
}


// slave's haptic loop
void *slaveThread(void *arg)
{
	// retrieve the device index as argument
	int slave = *((int*)arg);

	double sq[N]; // output torque for gravity compensation
	double sRefTime = dhdGetTime();
	double sTempTime;
	double sCycleTime;
	int sDelayIndex = 0;

	double svEst[N][N] = { 0.0 };

	// loop while the button is not pushed
	while (!done) {

		// get slave's position and velocity
		dhdGetDeltaJointAngles(&sj[0], &sj[1], &sj[2], slave);
		dhdDeltaGravityJointTorques(sj[0], sj[1], sj[2], &sq[0], &sq[1], &sq[2], slave);

		sTempTime = dhdGetTime();
		sCycleTime = sTempTime - sRefTime;
		sRefTime = sTempTime;

		// calculate master's velocity
		for (int i = 0; i < N; i++) {
			if (sDelayIndex == 0)	svEst[0][i] = (sj[i] - sjBack[DELAYCONSTMAX - 1][i]) / sCycleTime;
			else	svEst[0][i] = (sj[i] - sjBack[sDelayIndex - 1][i]) / sCycleTime;
			sv[i] = velocityCalculator(svBackv[0][i], svBackv[1][i], svEst[0][i], svEst[1][i], svEst[2][i], sCycleTime);
			svBackv[1][i] = svBackv[0][i];
			svBackv[0][i] = sv[i];
			svEst[2][i] = svEst[1][i];
			svEst[1][i] = svEst[0][i];
		}

		// set backward position and velocity
		for (int i = 0; i < N; i++) {
			if (sDelayIndex - delayConst < 0) {
				sjDelay[i] = sjBack[sDelayIndex + DELAYCONSTMAX - delayConst][i];
				svDelay[i] = svBack[sDelayIndex + DELAYCONSTMAX - delayConst][i];
			}
			else {
				sjDelay[i] = sjBack[sDelayIndex - delayConst][i];
				svDelay[i] = svBack[sDelayIndex - delayConst][i];
			}
			sjBack[sDelayIndex][i] = sj[i];
			svBack[sDelayIndex][i] = sv[i];
		}

		// calculate slave's joint torque
		st[0] = Kv * (mvDelay[0] - sv[0]) - Kd * sv[0] + Kp * (mjDelay[0] - sj[0]) + sq[0];
		st[1] = Kv * (mvDelay[2] - sv[1]) - Kd * sv[1] + Kp * (mjDelay[2] - sj[1]) + sq[1];
		st[2] = Kv * (mvDelay[1] - sv[2]) - Kd * sv[2] + Kp * (mjDelay[1] - sj[2]) + sq[2];

		dhdSetDeltaJointTorques(st[0], st[1], st[2], slave);
		dhdGetPosition(&sp[0], &sp[1], &sp[2], slave);

		if (sDelayIndex == DELAYCONSTMAX - 1) sDelayIndex = 0;
		else sDelayIndex++;

	}

	// close the connection
	drdClose(slave);
	return NULL;
}


int
main(int  argc,
	char **argv)
{
	int    master, slave;

	// message
	int major, minor, release, revision;
	dhdGetSDKVersion(&major, &minor, &release, &revision);
	printf("Force Dimension - Master Slave Example %d.%d.%d.%d\n", major, minor, release, revision);
	printf("(C) 2001-2015 Force Dimension\n");
	printf("All Rights Reserved.\n\n");

	// open and initialize 2 devices
	for (int dev = 0; dev < 2; dev++) {

		// open device
		if (drdOpenID(dev) < 0) {
			printf("error: not enough devices found\n");
			dhdSleep(2.0);
			for (int j = 0; j <= dev; j++) drdClose(j);
			return -1;
		}

		// exclude some device types that have not been fully tested with 'mirror'
		bool incompatible = false;
		switch (dhdGetSystemType()) {
		case DHD_DEVICE_SIGMA331:
		case DHD_DEVICE_SIGMA331_LEFT:
			incompatible = true;
			break;
		}

		// check that device is supported
		if (incompatible || !drdIsSupported()) {
			printf("error: unsupported device (%s)\n", dhdGetSystemName(dev));
			dhdSleep(2.0);
			for (int j = 0; j <= dev; j++) drdClose(j);
			return -1;
		}

		// initialize Falcon by hand if necessary
		if (!drdIsInitialized() && dhdGetSystemType() == DHD_DEVICE_FALCON) {
			printf("please initialize Falcon device...\r"); fflush(stdout);
			while (!drdIsInitialized()) dhdSetForce(0.0, 0.0, 0.0);
			printf("                                  \r");
			dhdSleep(0.5);
		}

		// initialize if necessary
		if (!drdIsInitialized(dev) && (drdAutoInit(dev) < 0)) {
			printf("error: initialization failed (%s)\n", dhdErrorGetLastStr());
			dhdSleep(2.0);
			for (int j = 0; j <= dev; j++) drdClose(j);
			return -1;
		}

		// start robot control loop
		if (drdStart(dev) < 0) {
			printf("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr());
			dhdSleep(2.0);
			for (int j = 0; j <= dev; j++) drdClose(j);
			return -1;
		}
	}

	// default role assignment
	master = 0;
	slave = 1;

	// prefer Falcon as master 
	if (dhdGetSystemType(0) != DHD_DEVICE_FALCON && dhdGetSystemType(1) == DHD_DEVICE_FALCON) {
		master = 1;
		slave = 0;
	}

	// give preference to omega.3 as slave
	if (dhdGetSystemType(0) == DHD_DEVICE_OMEGA3 && dhdGetSystemType(1) != DHD_DEVICE_OMEGA3) {
		master = 1;
		slave = 0;
	}

	// if a device is virtual, make it the master
	if (dhdGetComMode(1) == DHD_COM_MODE_VIRTUAL) {
		master = 1;
		slave = 0;
	}

	ushort mastersn, slavesn;
	dhdGetSerialNumber(&mastersn, master);
	dhdGetSerialNumber(&slavesn, slave);
	printf("%s haptic device [sn: %04d] as master\n", dhdGetSystemName(master), mastersn);
	printf("%s haptic device [sn: %04d] as slave\n\n", dhdGetSystemName(slave), slavesn);

	dhdEnableExpertMode();

	// center both devices
	//drdMoveToPos(0.0, 0.0, 0.0, false, master);
	//drdMoveToPos(0.0, 0.0, 0.0, true, slave);
	drdMoveToEnc(ENC, ENC, ENC, false, master);
	drdMoveToEnc(ENC, ENC, ENC, true, slave);
	while (drdIsMoving(master) || drdIsMoving(slave)) drdWaitForTick(master);

	// stop regulation on master, stop motion filters on slave
	drdStop(true, master);
	dhdSetForce(0.0, 0.0, 0.0, master);
	drdStop(true, slave);
	dhdSetForce(0.0, 0.0, 0.0, slave);

	//drdEnableFilter(false, master);
	//drdEnableFilter(false, slave);

	dhdGetDeltaJointAngles(&mj[0], &mj[1], &mj[2], master);
	dhdGetDeltaJointAngles(&sj[0], &sj[1], &sj[2], slave);

	// initialize lists of backstep joint angle
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < DELAYCONSTMAX; j++) {
			mjBack[j][i] = mj[i];
			sjBack[j][i] = sj[i];
		}
	}

	// start mastr's haptic loop
	dhdStartThread(masterThread, &master, DHD_THREAD_PRIORITY_DEFAULT);

	// start slave's haptic loop
	dhdStartThread(slaveThread, &slave, DHD_THREAD_PRIORITY_DEFAULT);

	double curTime, refTime = dhdGetTime();


	// loop while the button is not pushed
	while (!done) {

		Kd = CYCLETIME * delayConst * Kp;

		curTime = dhdGetTime();
		if ((curTime - refTime) > 0.1) {

			refTime = curTime;

			// retrieve information to display
			printf("%+0.05f %+0.05f ", mv[0], sv[0]);
			//printf("%+0.05f %+0.05f %+0.05f ", st[0], st[1], st[2]);
			//printf("%0.03f ", *mfz / abs(*pz - *spz));
			printf("tau_rt = %0.04f [ms] | ", 2 * CYCLETIME * delayConst * 1.0e3);
			printf("mf = %0.03f [kHz] | sf = %0.03f [kHz] \r", dhdGetComFreq(0), dhdGetComFreq(1));


			if (dhdGetButtonMask(master)) done = 1;
			if (dhdKbHit()) {
				switch (dhdKbGet()) {
				case 'q': done = 1;   break;
				case ',': delayConst = MAX(delayConst - DELAYSTEP, 1);	break;
				case '.': delayConst = MIN(delayConst + DELAYSTEP, DELAYCONSTMAX);	break;
				}
			}
		}
	}

	// report exit cause
	printf("                                                                           \r");
	if (done == -1) printf("\nregulation finished abnormally on slave device\n");
	else            printf("\nexiting on user request\n");

	// close the connection
	//drdClose(slave);
	drdClose(master);

	// exit
	printf("\ndone.\n");
	return 0;

}
