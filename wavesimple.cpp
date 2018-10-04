//  (C) 2001-2015 Force Dimension
//  All Rights Reserved.
//
//  Version 3.6.0

#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "drdc.h"

//#include <engine.h>
#include <iostream>
#include <string>

//#pragma comment ( lib, "libmx.lib" )
//#pragma comment ( lib, "libeng.lib" )
//#pragma comment ( lib, "libmex.lib" )
//#pragma comment ( lib, "libmat.lib" )

#define MIN(x,y) (x)<(y)?(x):(y)
#define MAX(x,y) (x)>(y)?(x):(y)

#define N 3
#define CYCLETIME 0.137e-3 // 0.25e-3 for windows and 0.137 for linux
#define CYCLETIMEUPPERLIMIT 0.3e-3
#define CYCLETIMELOWERLIMIT 0.1e-5
#define ENC 20300 // encoder reading corresponding to origin

#define WAVEIMPEDANCE 0.1 // from 0.01 to 1.0; 0.05 is the min value to stablize the system at no-delay case 0.16
#define MAXWAVEIMPEDANCE 2.0
#define MINWAVEIMPEDANCE 0.01
#define WAVEIMPEDANCESTEP 0.01

#define DEFAULT_K 12.0  // 12.0
#define DELAYCONST 1 // one-way delay equals to delayconst*cycletime
#define DELAYCHANGESTEP 91
#define DELAYCONSTMAX 8000

#define ALPHA 1.0e-1 // velocity filter parameter
#define BETA 1.0e-4 // wave velocity filter parameter
#define LAMDA 5.0e-4

#define PI 3.1415926
#define FREQN 2.0e2
#define zeta 0.90

double mp[N] = { 0.0 };
double sp[N] = { 0.0 };
double mv[N] = { 0.0 };
double sv[N] = { 0.0 };
double mj[N] = { 0.0 };
double sj[N] = { 0.0 };

double um[N] = { 0.0 };
double vm[N] = { 0.0 };
double us[N] = { 0.0 };
double vs[N] = { 0.0 };

double umDelay[N] = { 0.0 }; // the transfered data
double vmDelay[N] = { 0.0 };
double usDelay[N] = { 0.0 };
double vsDelay[N] = { 0.0 };

double mjDelay[N] = { 0.0 };

int done = 0;

double Kp = DEFAULT_K; // 1e3;
double Kv = Kp / 100;
double b = WAVEIMPEDANCE;

int delayConst = DELAYCONST;

double mTotalTime = 0.0;
int mIndex = 0;

double sTotalTime = 0.0;
int sIndex = 0;

double omegan = 2 * PI * FREQN;

// function for calculating velocity
static void velocityCalculator(double *v, double *vBack, double *vEst, double time) {		
	double K = omegan / tan(omegan * time / 2);	
	double a0 = K * K + 4 * K * zeta * omegan + omegan * omegan;
	double a1 = 2 * omegan * omegan - 2 * K * K;
	double a2 = K * K - 4 * K * zeta * omegan + omegan * omegan;
	double b0 = omegan * omegan;
	double b1 = 2 * omegan * omegan;
	double b2 = omegan * omegan;
	
	for (int i = 0; i < N; i++) {
		v[i] = (b0 * vEst[i] + b1 * vEst[N + i] + b2 * vEst[2 * N + i] - a1 * vBack[i] - a2 * vBack[N + i]) / a0;
		//v[i] = vEst[i] * ALPHA + vBack[i] * (1 - ALPHA);
		vBack[N + i] = vBack[i];
		vBack[i] = v[i];
		vEst[2 * N + i]	= vEst[N + i];
		vEst[N + i] = vEst[i];
	}
	
}

// master's haptic loop
void *masterThread(void *arg)
{
	int master = *((int*)arg);

	double mjvBack[N];
	double mvBackv[2 * N] = { 0.0 };
	double mvEst[N * N] = { 0.0 };

	double mjBack[DELAYCONSTMAX][N];
	
	double vmv[N] = { 0.0 };
	double vmvBack[N] = { 0.0 };

	double umBack[DELAYCONSTMAX][N] = { 0.0 };
	double vmBack[DELAYCONSTMAX][N] = { 0.0 };

	double mt[N];
	double mq[N];

	double mTempTime;
	double mLastTempTime;
	double mCycleTime;

	int mDelayIndex = 0;

	dhdGetDeltaJointAngles(&mj[0], &mj[1], &mj[2], master);

	for (int i = 0; i < N; i++) {
		mjvBack[i] = mj[i];
		for(int j = 0; j < DELAYCONST; j++) {
			mjBack[j][i] = mj[i];	
		}
	}

	mIndex = 1;
	while (!sIndex) {

	}

	mLastTempTime = dhdGetTime();

	while (!done) {

		mTotalTime = delayConst * CYCLETIME;

		// get master's position and velocity
		dhdGetDeltaJointAngles(&mj[0], &mj[1], &mj[2], master);
		dhdDeltaGravityJointTorques(mj[0], mj[1], mj[2], &mq[0], &mq[1], &mq[2], master);

		mTempTime = dhdGetTime();
		mCycleTime = mTempTime - mLastTempTime;
		mLastTempTime = mTempTime;

		// calculate master's velocity
		for (int i = 0; i < N; i++) {
			mvEst[i] = (mj[i] - mjvBack[i]) / mCycleTime;
			mjvBack[i] = mj[i];
		}
		
		velocityCalculator(mv, mvBackv, mvEst, mCycleTime);

		for (int i = 0; i < N; i++) {
			if (mDelayIndex == 0) vmv[i] = ((vsDelay[i] - vmBack[DELAYCONSTMAX - 1][i]) / mCycleTime * BETA + (1 - BETA)*vmvBack[i]) / (1 + sTotalTime / mCycleTime * BETA);
			else vmv[i] = ((vsDelay[i] - vmBack[mDelayIndex - 1][i]) / mCycleTime * BETA + (1 - BETA)*vmvBack[i]) / (1 + sTotalTime / mCycleTime * BETA);
			vmvBack[i] = vmv[i];
			vm[i] = vsDelay[i] - vmv[i] * sTotalTime;
			mt[i] = -(b * mv[i] - sqrt(2 * b)*vm[i]);
			um[i] = sqrt(2 * b)*mv[i] - vm[i];
			
		}

		dhdSetDeltaJointTorques(mt[0] + mq[0], mt[1] + mq[1], mt[2] + mq[2], master);

		dhdGetPosition(&mp[0], &mp[1], &mp[2], master);

		// set backward wave
		for (int i = 0; i < N; i++) {
			if (mDelayIndex - delayConst < 0) {
				umDelay[i] = umBack[mDelayIndex + DELAYCONSTMAX - delayConst][i];
				vmDelay[i] = vmBack[mDelayIndex + DELAYCONSTMAX - delayConst][i];
				mjDelay[i] = mjBack[mDelayIndex + DELAYCONSTMAX - delayConst][i];
			}
			else {
				umDelay[i] = umBack[mDelayIndex - delayConst][i];
				vmDelay[i] = vmBack[mDelayIndex - delayConst][i];
				mjDelay[i] = mjBack[mDelayIndex - delayConst][i];
			}
			umBack[mDelayIndex][i] = um[i];
			vmBack[mDelayIndex][i] = vm[i];
			mjBack[mDelayIndex][i] = mj[i];
		}

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

	double sjvBack[N];
	double svBackv[2 * N] = { 0.0 };
	double svEst[N * N] = { 0.0 };
	
	double svd[N] = { 0.0 };
	double sjd[N];	

	double usv[N] = { 0.0 };
	double usvBack[N] = { 0.0 };

	double usBack[DELAYCONSTMAX][N] = { 0.0 };
	double vsBack[DELAYCONSTMAX][N] = { 0.0 };

	double sjdBackStep[N];

	double st[N] = { 0.0 };
	double sq[N];

	double sTempTime;
	double sLastTempTime;
	double sCycleTime;

	int sDelayIndex = 0;

	dhdGetDeltaJointAngles(&sj[0], &sj[1], &sj[2], slave);

	for (int i = 0; i < N; i++) {
		sjd[i] = sj[i];
		sjvBack[i] = sj[i];
		sjdBackStep[i] = sj[i];
	}

	sIndex = 1;
	while (!mIndex) {

	}

	sLastTempTime = dhdGetTime();

	// loop while the button is not pushed
	while (!done) {

		sTotalTime = delayConst * CYCLETIME;

		// get slave's position and velocity
		dhdGetDeltaJointAngles(&sj[0], &sj[1], &sj[2], slave);
		dhdDeltaGravityJointTorques(sj[0], sj[1], sj[2], &sq[0], &sq[1], &sq[2], slave);

		sTempTime = dhdGetTime();
		sCycleTime = sTempTime - sLastTempTime;
		sLastTempTime = sTempTime;

		// calculate master's velocity
		for (int i = 0; i < N; i++) {
			svEst[i] = (sj[i] - sjvBack[i]) / sCycleTime;
			sjvBack[i] = sj[i];
		}

		velocityCalculator(sv, svBackv, svEst, sCycleTime);

		for (int i = 0; i < N; i++) {
			if (sDelayIndex == 0) usv[i] = ((umDelay[i] - usBack[DELAYCONSTMAX - 1][i]) / sCycleTime * BETA + (1 - BETA)*usvBack[i]) / (1 + mTotalTime / sCycleTime * BETA);
			else usv[i] = ((umDelay[i] - usBack[sDelayIndex - 1][i]) / sCycleTime * BETA + (1 - BETA)*usvBack[i]) / (1 + mTotalTime / sCycleTime * BETA);
			usvBack[i] = usv[i];
			us[i] = umDelay[i] - usv[i] * mTotalTime;		
			sjd[i] += svd[i] * sCycleTime + (mjDelay[i] - sjdBackStep[i]) * LAMDA;
			svd[i] = (sqrt(2 * b)*us[i] + Kv * sv[i] + Kp * (sj[i] - sjd[i])) / (Kv + b);
			sjdBackStep[i] = sjd[i];
			st[i] = Kv * (svd[i] - sv[i]) + Kp * (sjd[i] - sj[i]);
			vs[i] = us[i] - sqrt(2 / b)*st[i];
		}

		dhdSetDeltaJointTorques(st[0] + sq[0], st[1] + sq[1], st[2] + sq[2], slave);

		dhdGetPosition(&sp[0], &sp[1], &sp[2], slave);

		// set backward position and velocity
		for (int i = 0; i < N; i++) {
			if (sDelayIndex - delayConst < 0) {
				usDelay[i] = usBack[sDelayIndex + DELAYCONSTMAX - delayConst][i];
				vsDelay[i] = vsBack[sDelayIndex + DELAYCONSTMAX - delayConst][i];
			}
			else {
				usDelay[i] = usBack[sDelayIndex - delayConst][i];
				vsDelay[i] = vsBack[sDelayIndex - delayConst][i];
			}
			usBack[sDelayIndex][i] = us[i];
			vsBack[sDelayIndex][i] = vs[i];
		}
		
		if (sDelayIndex == DELAYCONSTMAX - 1) sDelayIndex = 0;
		else sDelayIndex++;

	}

	// close the connection
	//drdClose(slave);
	return NULL;
}

int
main(int  argc,
	char **argv)
{
	int    master, slave;
	double timeDelay;

	// message
	int major, minor, release, revision;
	dhdGetSDKVersion(&major, &minor, &release, &revision);
	//printf("Force Dimension - Master Slave Example %d.%d.%d.%d\n", major, minor, release, revision);
	//printf("(C) 2001-2015 Force Dimension\n");
	//printf("All Rights Reserved.\n\n");

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

	// center both devices
	drdMoveToEnc(ENC, ENC, ENC, false, master);
	drdMoveToEnc(ENC - 150, ENC, ENC, true, slave);
	while (drdIsMoving(master) || drdIsMoving(slave)) drdWaitForTick(master);

	// stop regulation on master, stop motion filters on slave
	drdStop(true, master);
	dhdSetForce(0.0, 0.0, 0.0, master);
	drdStop(true, slave);
	dhdSetForce(0.0, 0.0, 0.0, slave);

	dhdEnableExpertMode();

	//drdEnableFilter(false, master);
	//drdEnableFilter(false, slave);

	// start slave's haptic loop
	dhdStartThread(slaveThread, &slave, DHD_THREAD_PRIORITY_DEFAULT);

	// start mastr's haptic loop
	dhdStartThread(masterThread, &master, DHD_THREAD_PRIORITY_DEFAULT);

	double curTime, refTime = dhdGetTime();

	/*
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

	while (!(mIndex&&sIndex)) {

	}

	*px = -mp[0];
	*py = mp[1];
	*pz = mp[2];
	*spx = -sp[0];
	*spy = sp[1];
	*spz = sp[2];
	*/

	printf("round trip delay    wave impedance \n");

	// loop while the button is not pushed
	while (!done) {
		/*
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

		// matlab plot
		*px = -mp[0];
		*py = mp[1];
		*pz = mp[2];
		*spx = -sp[0];
		*spy = sp[1];
		*spz = sp[2];

		engPutVariable(m_pEngine, "dz", dz);
		engPutVariable(m_pEngine, "sdz", sdz);

		engPutVariable(m_pEngine, "mforcez", mforcez);
		engPutVariable(m_pEngine, "sforcez", sforcez);

		*t = dhdGetTime() - t1;
		engPutVariable(m_pEngine, "t", time);
		
		engEvalString(m_pEngine, "subplot(2,2,1),plot([lt,t],[ldz,dz],'b'),hold on,plot([lt,t],[lsdz,sdz],'r'),");
		engEvalString(m_pEngine, "subplot(2,2,2),plot([lt,t],[lmforcez,mforcez],'b'),hold on,plot([lt,t],[-lsforcez,-sforcez],'r'),");
		engEvalString(m_pEngine, "subplot(2,2,3),plot([lt,t],[abs(ldz-lsdz),abs(dz-sdz)],'b'),hold on,");
		engEvalString(m_pEngine, "subplot(2,2,4),plot([lt,t],[abs(lmforcez+lsforcez),abs(mforcez+sforcez)],'r'),hold on,");*/

		curTime = dhdGetTime();
		if ((curTime - refTime) > 0.1) {

			refTime = curTime;

			timeDelay = (mTotalTime + sTotalTime)*1.0e3;

			// retrieve information to display
			//printf("%+0.03f %+0.03f %+0.03f | ", mj[0], mj[1], mj[2]);
			//printf("%+0.03f %+0.03f %+0.03f | ", sj[0], sj[1], sj[2]);
			printf("tau_rt = %0.02f [ms] | ", timeDelay);
			printf("b = %0.03f | ", b);
			printf("mf = %0.03f [kHz] | sf = %0.03f [kHz] \r", dhdGetComFreq(0), dhdGetComFreq(1));

			if (dhdGetButtonMask(master)) done = 1;
			if (dhdKbHit()) {
				switch (dhdKbGet()) {
				case 'q': done = 1;   break;
				case ',': delayConst = MAX(delayConst - DELAYCHANGESTEP, 1);	break;
				case '.': delayConst = MIN(delayConst + DELAYCHANGESTEP, DELAYCONSTMAX);	break;
				case 'n': b = MAX(b - WAVEIMPEDANCESTEP, MINWAVEIMPEDANCE);	break;
				case 'm': b = MIN(b + WAVEIMPEDANCESTEP, MAXWAVEIMPEDANCE);	break;
				}
			}
		}

	}
	/*
	*td = 2 * 0.25*delayConst;
	*kgain = Kp;
	engPutVariable(m_pEngine, "delay", delay);
	engPutVariable(m_pEngine, "kpgain", kpgain);

	engEvalString(m_pEngine, "subplot(2,2,1),grid on,xlabel('time(s)'),ylabel('position(m)'),title(['master and slave z-position: tau_{rt} =' num2str(delay) 'ms, kp = ' num2str(kpgain) 'N*m/rad']),");
	engEvalString(m_pEngine, "subplot(2,2,2),grid on,xlabel('time(s)'),ylabel('force(N)'),title(['master and slave z-force: tau_{rt} =' num2str(delay) 'ms, kp = ' num2str(kpgain) 'N*m/rad']),");
	engEvalString(m_pEngine, "subplot(2,2,3),grid on,xlabel('time(s)'),ylabel('position(m)'),title(['master and slave z-position error: tau_{rt} =' num2str(delay) 'ms, kp = ' num2str(kpgain) 'N*m/rad']),");
	engEvalString(m_pEngine, "subplot(2,2,4),grid on,xlabel('time(s)'),ylabel('force(N)'),title(['master and slave z-force error: tau_{rt} =' num2str(delay) 'ms, kp = ' num2str(kpgain) 'N*m/rad']),");
	*/
	// report exit cause
	printf("                                                                           \r");
	if (done == -1) printf("\nregulation finished abnormally on slave device\n");
	else            printf("\nexiting on user request\n");

	// close the connection
	drdClose(slave);
	drdClose(master);

	// exit
	printf("\ndone.\n");
	return 0;

}
