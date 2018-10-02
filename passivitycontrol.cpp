//  (C) 2001-2015 Force Dimension
//  All Rights Reserved.
//
//  Version 3.6.0

#include "stdafx.h"

#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "drdc.h"

#include <engine.h>
#include <iostream>
#include <string>

#pragma comment ( lib, "libmx.lib" )
#pragma comment ( lib, "libeng.lib" )
#pragma comment ( lib, "libmex.lib" )
#pragma comment ( lib, "libmat.lib" )

#define MIN(x,y) (x)<(y)?(x):(y)
#define MAX(x,y) (x)>(y)?(x):(y)

#define DEFAULT_K 12.0  // 1.0 12.0 14.0 16.0
#define N 3
#define DELAYCONST 1 // max 2001 141 121 101
#define DELAYCONSTMAX 141
#define CYCLETIME 0.25e-3
#define ALPHA 0.08 // 0.08
#define ENC 20300 // encoder reading corresponding to origin

double mp[N] = { 0.0 }; // end-effector position
double sp[N] = { 0.0 };

double mj[N] = { 0.0 }; // joint angle
double sj[N] = { 0.0 };

double mv[N] = { 0.0 }; // angular velocity
double sv[N] = { 0.0 };

double mvbackv[N] = { 0.0 }; // last step angular velocity
double svbackv[N] = { 0.0 };

double mjback[DELAYCONSTMAX][N] = { 0.0 }; // list of backstep joint angle to simulate delay
double sjback[DELAYCONSTMAX][N] = { 0.0 };

double mvback[DELAYCONSTMAX][N] = { 0.0 }; //list of backstep angular velocity to simulate delay
double svback[DELAYCONSTMAX][N] = { 0.0 };

double mjDelay[N];
double mvDelay[N];

double sjDelay[N];
double svDelay[N];

double mt[N]; // output torque
double st[N];

double Kp = DEFAULT_K;
double Kv;
double Kd;

int delayConst = DELAYCONST;

int    done = 0;

// master's haptic loop
void *masterThread(void *arg)
{
	int master = *((int*)arg);

	double mq[N]; // output torque for gravity compensation
	double mRefTime = dhdGetTime();
	double mTempTime;
	double mCycleTime;
	int mDelayIndex = 0;

	while (!done) {

		// increase kgain to target value
		if (Kp < DEFAULT_K) {
			Kp += Kp / 400;
			Kv = Kp / 800;
			Kd = 0.25e-3*(delayConst - 1)*Kp;
		}
		else {
			Kp = DEFAULT_K;
			Kv = Kp / 800;
			Kd = 0.25e-3*(delayConst - 1)*Kp;
		}

		// get master's position and velocity
		dhdGetDeltaJointAngles(&mj[0], &mj[1], &mj[2], master);
		dhdDeltaGravityJointTorques(mj[0], mj[1], mj[2], &mq[0], &mq[1], &mq[2], master);

		mTempTime = dhdGetTime();
		mCycleTime = mTempTime - mRefTime;
		mRefTime = mTempTime;

		// calculate master's velocity
		for (int i = 0; i < N; i++) {
			if (mDelayIndex == 0) mv[i] = (mj[i] - mjback[DELAYCONSTMAX - 1][i]) / mCycleTime * ALPHA + (1 - ALPHA)*mvbackv[i];
			else mv[i] = (mj[i] - mjback[mDelayIndex - 1][i]) / mCycleTime * ALPHA + (1 - ALPHA)*mvbackv[i];
			mvbackv[i] = mv[i];
		}

		// set backward position and velocity
		for (int i = 0; i < N; i++) {
			if (mDelayIndex - delayConst < 0) {
				mjDelay[i] = mjback[mDelayIndex + DELAYCONSTMAX - delayConst][i];
				mvDelay[i] = mvback[mDelayIndex + DELAYCONSTMAX - delayConst][i];
			}
			else {
				mjDelay[i] = mjback[mDelayIndex - delayConst][i];
				mvDelay[i] = mvback[mDelayIndex - delayConst][i];
			}
			mjback[mDelayIndex][i] = mj[i];
			mvback[mDelayIndex][i] = mv[i];
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
			if (sDelayIndex == 0) sv[i] = (sj[i] - sjback[DELAYCONSTMAX - 1][i]) / sCycleTime * ALPHA + (1 - ALPHA)*svbackv[i];
			else sv[i] = (sj[i] - sjback[sDelayIndex - 1][i]) / sCycleTime * ALPHA + (1 - ALPHA)*svbackv[i];
			svbackv[i] = sv[i];
		}

		// set backward position and velocity
		for (int i = 0; i < N; i++) {
			if (sDelayIndex - delayConst < 0) {
				sjDelay[i] = sjback[sDelayIndex + DELAYCONSTMAX - delayConst][i];
				svDelay[i] = svback[sDelayIndex + DELAYCONSTMAX - delayConst][i];
			}
			else {
				sjDelay[i] = sjback[sDelayIndex - delayConst][i];
				svDelay[i] = svback[sDelayIndex - delayConst][i];
			}
			sjback[sDelayIndex][i] = sj[i];
			svback[sDelayIndex][i] = sv[i];
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

	Kp = Kp / 4; // start kgain with a quarter of default k
	Kv = Kp / 800;
	Kd = 0.25e-3*(delayConst - 1)*Kp;

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
			mjback[j][i] = mj[i];
			sjback[j][i] = sj[i];
		}
	}

	// start mastr's haptic loop
	dhdStartThread(masterThread, &master, DHD_THREAD_PRIORITY_DEFAULT);

	// start slave's haptic loop
	dhdStartThread(slaveThread, &slave, DHD_THREAD_PRIORITY_DEFAULT);

	double curTime, refTime = dhdGetTime();

	// open matlab engine
	Engine *m_pEngine;
	m_pEngine = engOpen("null");

	mxArray* dx = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* dy = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* dz = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* sdx = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* sdy = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* sdz = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* time = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* mforcez = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* sforcez = mxCreateDoubleMatrix(1, 1, mxREAL);

	mxArray* ldx = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* ldy = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* ldz = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* lsdx = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* lsdy = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* lsdz = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* ltime = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* lmforcez = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* lsforcez = mxCreateDoubleMatrix(1, 1, mxREAL);

	mxArray* delay = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* kpgain = mxCreateDoubleMatrix(1, 1, mxREAL);

	double* px = mxGetPr(dx);
	double* py = mxGetPr(dy);
	double* pz = mxGetPr(dz);
	double* spx = mxGetPr(sdx);
	double* spy = mxGetPr(sdy);
	double* spz = mxGetPr(sdz);
	double* t = mxGetPr(time);
	double* mfz = mxGetPr(mforcez);
	double* sfz = mxGetPr(sforcez);

	double* lpx = mxGetPr(ldx);
	double* lpy = mxGetPr(ldy);
	double* lpz = mxGetPr(ldz);
	double* lspx = mxGetPr(lsdx);
	double* lspy = mxGetPr(lsdy);
	double* lspz = mxGetPr(lsdz);
	double* lt = mxGetPr(ltime);
	double* lmfz = mxGetPr(lmforcez);
	double* lsfz = mxGetPr(lsforcez);

	double* td = mxGetPr(delay);
	double* kgain = mxGetPr(kpgain);

	double t1 = dhdGetTime();
	double mfy, mfx;
	double sfy, sfx;

	// loop while the button is not pushed
	while (!done) {

		*lpz = *pz;
		*lspz = *spz;
		*lmfz = *mfz;
		*lsfz = *sfz;
		*lt = *t;

		engPutVariable(m_pEngine, "ldz", ldz);
		engPutVariable(m_pEngine, "lsdz", lsdz);

		engPutVariable(m_pEngine, "lmforcez", lmforcez);
		engPutVariable(m_pEngine, "lsforcez", lsforcez);

		engPutVariable(m_pEngine, "lt", ltime);

		dhdGetForce(&mfx, &mfy, mfz, master);
		dhdGetForce(&sfx, &sfy, sfz, slave);
		//dhdGetDeltaEncoders(&menc[0], &menc[1], &menc[2], master);
		//dhdGetDeltaEncoders(&senc[0], &senc[1], &senc[2], slave);

		// matlab plot
		*px = -mp[0];
		*py = mp[1];
		*pz = mp[2];
		*spx = -sp[0];
		*spy = sp[1];
		*spz = sp[2];

		//engPutVariable(m_pEngine, "dx", dx);
		//engPutVariable(m_pEngine, "dy", dy);
		engPutVariable(m_pEngine, "dz", dz);
		//engPutVariable(m_pEngine, "sdx", sdx);
		//engPutVariable(m_pEngine, "sdy", sdy);
		engPutVariable(m_pEngine, "sdz", sdz);

		engPutVariable(m_pEngine, "mforcez", mforcez);
		engPutVariable(m_pEngine, "sforcez", sforcez);

		*t = dhdGetTime() - t1;
		engPutVariable(m_pEngine, "t", time);

		//engEvalString(m_pEngine, "scatter3(dx,dy,dz,'b','filled'),set(gca,'Xlim',[-0.05 0.05],'Ylim',[-0.05 0.05],'Zlim',[-0.05 0.05]),");
		//engEvalString(m_pEngine, "hold on,scatter3(sdx,sdy,sdz,'r','filled'),");
		//engEvalString(m_pEngine, "xlabel('x'),ylabel('y'),zlabel('z'),hold off");
		engEvalString(m_pEngine, "subplot(2,2,1),plot([lt,t],[ldz,dz],'b'),hold on,plot([lt,t],[lsdz,sdz],'r'),");
		engEvalString(m_pEngine, "subplot(2,2,2),plot([lt,t],[lmforcez,mforcez],'b'),hold on,plot([lt,t],[-lsforcez,-sforcez],'r'),");
		engEvalString(m_pEngine, "subplot(2,2,3),plot([lt,t],[abs(ldz-lsdz),abs(dz-sdz)],'b'),hold on,");
		engEvalString(m_pEngine, "subplot(2,2,4),plot([lt,t],[abs(lmforcez+lsforcez),abs(mforcez+sforcez)],'r'),hold on,");

		curTime = dhdGetTime();
		if ((curTime - refTime) > 0.1) {

			refTime = curTime;

			// retrieve information to display
			//printf("%+0.05f %+0.05f %+0.05f ", mt[0], mt[1], mt[2]);
			//printf("%+0.05f %+0.05f %+0.05f ", st[0], st[1], st[2]);
			//printf("%0.03f ", *mfz / abs(*pz - *spz));
			printf("tau_rt = %0.02f [ms] | ", 2 * 0.25*(delayConst - 1));
			printf("mf = %0.03f [kHz] | sf = %0.03f [kHz] \r", dhdGetComFreq(0), dhdGetComFreq(1));


			if (dhdGetButtonMask(master)) done = 1;
			if (dhdKbHit()) {
				switch (dhdKbGet()) {
				case 'q': done = 1;   break;
				case ',': delayConst = MAX(delayConst - 1, 1);	break;
				case '.': delayConst = MIN(delayConst + 1, DELAYCONSTMAX);	break;
				}
			}
		}
	}

	*td = 2 * 0.25*(delayConst - 1);
	*kgain = Kp;
	engPutVariable(m_pEngine, "delay", delay);
	engPutVariable(m_pEngine, "kpgain", kpgain);

	engEvalString(m_pEngine, "subplot(2,2,1),grid on,xlabel('time(s)'),ylabel('position(m)'),title(['master and slave z-position: tau_{rt} =' num2str(delay) 'ms, kp = ' num2str(kpgain) 'N*m/rad']),");
	engEvalString(m_pEngine, "subplot(2,2,2),grid on,xlabel('time(s)'),ylabel('force(N)'),title(['master and slave z-force: tau_{rt} =' num2str(delay) 'ms, kp = ' num2str(kpgain) 'N*m/rad']),");
	engEvalString(m_pEngine, "subplot(2,2,3),grid on,xlabel('time(s)'),ylabel('position(m)'),title(['master and slave z-position error: tau_{rt} =' num2str(delay) 'ms, kp = ' num2str(kpgain) 'N*m/rad']),");
	engEvalString(m_pEngine, "subplot(2,2,4),grid on,xlabel('time(s)'),ylabel('force(N)'),title(['master and slave z-force error: tau_{rt} =' num2str(delay) 'ms, kp = ' num2str(kpgain) 'N*m/rad']),");

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
