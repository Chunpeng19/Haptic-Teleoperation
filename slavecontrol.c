#include "drd.h"
#include <math.h>
#include <iostream>
#include <string>

#define N 3
#define DELAYCONSTMAX 8000
#define CYCLETIME 0.137e-3

// slave's haptic loop
void *slaveControl(void *arg)
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

	double sjdBack[N];

	double st[N] = { 0.0 };
	double sq[N];

	double sTempTime;
	double sLastTempTime;
	double sCycleTime;

	double usBack[DELAYCONSTMAX * N] = { 0.0 };
	double vsBack[DELAYCONSTMAX * N] = { 0.0 };

	int sDelayIndex = 0;

	drdMoveToEnc(ENC, ENC, ENC, false, slave);
	while (drdIsMoving(slave))
	drdStop(true, slave);
	dhdSetForce(0.0, 0.0, 0.0, slave);

	dhdGetDeltaJointAngles(&sj[0], &sj[1], &sj[2], slave);

	for (int i = 0; i < N; i++) {
		sjd[i] = sj[i];
		sjvBack[i] = sj[i];
		sjdBack[i] = sj[i];
	}


	sIndex = 1;
	while (!mIndex)

	sLastTempTime = dhdGetTime();

	// loop while the button is not pushed
	while (!done) {

		sTotalTime = delayConst * CYCLETIME;

		// get position and velocity
		dhdGetDeltaJointAngles(&sj[0], &sj[1], &sj[2], slave);
		dhdDeltaGravityJointTorques(sj[0], sj[1], sj[2], &sq[0], &sq[1], &sq[2], slave);
		
		// calculate loop time
		sTempTime = dhdGetTime();
		sCycleTime = sTempTime - sLastTempTime;
		sLastTempTime = sTempTime;

		// calculate velocity
		velocityCalculator(sv, svBackv, svEst, sj, sjvBack, sCycleTime);
		
		// calulate wave velocity
		waveVelocityCalculator(usv, usvBack, usBack, umDelay, sCycleTime, mTotalTime, sDelayIndex);
		
		// decode and encode wave variable
		slaveWaveDecodeEncode(us, vs, usv, umDelay, st, sv, sj, svd, sjd, sjdBack, mjDelay, sCycleTime, mTotalTime);
		
		// set joint torque
		dhdSetDeltaJointTorques(st[0] + sq[0], st[1] + sq[1], st[2] + sq[2], slave);
		
		// get Cartisian position
		dhdGetPosition(&sp[0], &sp[1], &sp[2], slave);

		// set backward position and velocity
		delayMimic(usDelay, usBack, us, sDelayIndex);
		delayMimic(vsDelay, vsBack, vs, sDelayIndex);		
		if (sDelayIndex == DELAYCONSTMAX - 1) sDelayIndex = 0;
		else sDelayIndex++;


	}

	// close the connection
	drdClose(slave);
	return NULL;
}
