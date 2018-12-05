#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "drdc.h"
#include <iostream>
#include <string>
#include "engine.h"

#define N 3
#define TESTTORQUE -0.35
#define IMPULSECYCLES 100
#define TESTEDJOINT 2

double p[N] = { 0.0 }; // end-effector position

double j[N] = { 0.0 }; // joint angle

double v[N] = { 0.0 }; // angular velocity

double t[N] = { 0.0 }; // output torque

double minq[N];
double maxq[N];

int flag = 0;
int done = 0;

// control loop
void *controlThread(void *arg)
{
	double q[N];
	int cycle = IMPULSECYCLES;

	while(!done) {
		
		// get joint angle and
		dhdGetDeltaJointAngles(&j[0],&j[1],&j[2]);
		dhdDeltaGravityJointTorques(j[0], j[1], j[2], &q[0], &q[1], &q[2]);

		if(flag) t[TESTEDJOINT] = TESTTORQUE;
		else t[TESTEDJOINT] = 0.0;

		dhdSetDeltaJointTorques(t[0] + q[0], t[1] + q[1], t[2] + q[2]);

		if(t[TESTEDJOINT] == TESTTORQUE) cycle--;
		if(cycle == 0) {
			flag = 0;
			cycle = IMPULSECYCLES;		
		}
	}
	
	printf("control thread stop.\n");
	pthread_exit(NULL);
	return NULL;
}

// plot loop
void *plotThread(void *arg)
{
	// open matlab engine
	Engine *m_pEngine;
	if (!(m_pEngine = engOpen(""))) {
		printf("Error: cannot start MATLAB engine...\n");	
	}

	mxArray* dx = mxCreateDoubleMatrix(1, 1, mxREAL);
	//mxArray* dy = mxCreateDoubleMatrix(1, 1, mxREAL);
	//mxArray* dz = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* time = mxCreateDoubleMatrix(1, 1, mxREAL);

	mxArray* ldx = mxCreateDoubleMatrix(1, 1, mxREAL);
	//mxArray* ldy = mxCreateDoubleMatrix(1, 1, mxREAL);
	//mxArray* ldz = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray* ltime = mxCreateDoubleMatrix(1, 1, mxREAL);

	double* px = mxGetPr(dx);
	//double* py = mxGetPr(dy);
	//double* pz = mxGetPr(dz);
	double* t = mxGetPr(time);

	double* lpx = mxGetPr(ldx);
	//double* lpy = mxGetPr(ldy);
	//double* lpz = mxGetPr(ldz);
	double* lt = mxGetPr(ltime);

	double t1 = dhdGetTime();

	*px = -j[TESTEDJOINT];
	//*py = -j[1];
	//*pz = -j[2];

	while(!done) {

		*lt = *t;
		*lpx = *px;
		//*lpy = *py;
		//*lpz = *pz;
		
		engPutVariable(m_pEngine, "ldx", ldx);
		//engPutVariable(m_pEngine, "ldy", ldy);
		//engPutVariable(m_pEngine, "ldz", ldz);
		engPutVariable(m_pEngine, "lt", ltime);

		// matlab plot
		*px = -j[TESTEDJOINT];
		//*py = -j[1];
		//*pz = -j[2];

		engPutVariable(m_pEngine, "dx", dx);
		//engPutVariable(m_pEngine, "dy", dy);
		//engPutVariable(m_pEngine, "dz", dz);

		*t = dhdGetTime() - t1;
		engPutVariable(m_pEngine, "t", time);
		
		engEvalString(m_pEngine, "plot([lt,t],[ldx,dx],'b'),hold on,");

	}

	printf("plot thread stop.\n");
	pthread_exit(NULL);
	return NULL;

}

// data recording
void *dataThread(void *arg)
{
	double currentDataTime, refDataTime, deltaDataTime;
	int dataFlag = 0;
	int dataSteady = 500;
	double tempJ;
	
	FILE *f = fopen("/home/lowell/Documents/Haptic-Teleoperation/examples/systemidentification/data.txt", "w");

	if(f == NULL) {
		printf("Error opening file!\n");
		done = 1;	
	}

	//const char *text = "Data record start...";
	//fprintf(f, "%s\n", text);

	while(!done) {
		refDataTime = dhdGetTime();
		if(flag) dataFlag = 1;
		while(dataFlag) {
			tempJ = j[TESTEDJOINT];
			currentDataTime = dhdGetTime();
			deltaDataTime = currentDataTime - refDataTime;		
			fprintf(f, "%0.06f, %0.06f\n", deltaDataTime, j[TESTEDJOINT]);
			dhdSleep(0.001);
			if(j[TESTEDJOINT] == tempJ) --dataSteady;
			if(dataSteady == 0) dataFlag = 0;
		}		
	}
	
	fclose(f);
	printf("data record thread stop.\n");
	pthread_exit(NULL);
	return NULL;
}

int
main(int  argc,
	char **argv)
{
	bool kbHit = false;
	bool kbHitBack = false;
	
	// message
	int major, minor, release, revision;
	dhdGetSDKVersion(&major, &minor, &release, &revision);
	printf("Force Dimension - Master Slave Example %d.%d.%d.%d\n", major, minor, release, revision);
	printf("(C) 2001-2015 Force Dimension\n");
	printf("All Rights Reserved.\n\n");

	// open device
	if (drdOpen() < 0) {
		printf("error: not enough devices found\n");
		dhdSleep(2.0);
		return -1;
	}

	// check that device is supported
	if (!drdIsSupported()) {
		printf("error: unsupported device (%s)\n", dhdGetSystemName());
		dhdSleep(2.0);
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
	if (!drdIsInitialized() && (drdAutoInit() < 0)) {
		printf("error: initialization failed (%s)\n", dhdErrorGetLastStr());
		dhdSleep(2.0);
		return -1;
	}

	// start robot control loop
	if (drdStart() < 0) {
		printf("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr());
		dhdSleep(2.0);
		return -1;
	}

	ushort devsn;
	dhdGetSerialNumber(&devsn);
	printf("%s haptic device [sn: %04d] connected...\n", dhdGetSystemName(), devsn);

	// center both devices
	drdMoveToPos(0.0, 0.0, 0.0, true);

	// stop regulation on master, stop motion filters on slave
	drdStop(true);
	dhdSetForce(0.0, 0.0, 0.0);

	drdEnableFilter(false);

	dhdEnableExpertMode();

	printf("%s haptic device [sn: %04d] initialization completed...\n", dhdGetSystemName(), devsn);

	// thread create
	pthread_t control_thread, plot_thread, data_thread;
	
	if(pthread_create(&control_thread, NULL, controlThread, NULL)) {
		fprintf(stderr, "Error creating thread...\n");
		done = 1;	
	}

	if(pthread_create(&plot_thread, NULL, plotThread, NULL)) {
		fprintf(stderr, "Error creating thread...\n");
		done = 1;	
	}

	if(pthread_create(&data_thread, NULL, dataThread, NULL)) {
		fprintf(stderr, "Error creating thread...\n");
		done = 1;	
	}

	double curTime, refTime = dhdGetTime();

	// loop while the button is not pushed
	while (!done) {

		dhdDeltaJointTorquesExtrema(j[0], j[1], j[2], minq, maxq);

		curTime = dhdGetTime();
		if ((curTime - refTime) > 0.1) {

			refTime = curTime;

			// retrieve information to display
			printf("j = %+0.04f | %+0.04f | %+0.04f | ", minq[0], minq[1], minq[2]);
			printf("j = %+0.04f | %+0.04f | %+0.04f | \r", maxq[0], maxq[1], maxq[2]);
			//printf("f = %0.03f [kHz]\r", dhdGetComFreq());


			if (dhdGetButtonMask()) done = 1;
			kbHitBack = kbHit;
			kbHit = dhdKbHit();
			if (kbHit && (!kbHitBack)) {
				switch (dhdKbGet()) {
				case 'q': done = 1;   break;
				case 'a': flag = 1;   break;			
				}
			}
		}
	}

	// report exit cause
	printf("                                                                           \r");
	if (done == -1) printf("\nregulation finished abnormally on slave device...\n");
	else            printf("\nexiting on user request...\n");

	// close the connection
	drdClose();
	dhdSleep(1.0);

	// exit
	printf("\ndone.\n");
	return 0;

}
