#include "drd.h"
#include <math.h>
#include <iostream>
#include <string>

// master's haptic loop
void *masterThread(void *arg)
{
	int master = *((int*)arg);

	double mjvBack[N];
	double mvBackv[2 * N] = { 0.0 };
	double mvEst[N * N] = { 0.0 };	
	
	double vmv[N] = { 0.0 };
	double vmvBack[N] = { 0.0 };

	double mt[N];
	double mq[N];

	double mTempTime;
	double mLastTempTime;
	double mCycleTime;
	
	double umBack[DELAYCONSTMAX * N] = { 0.0 };
	double vmBack[DELAYCONSTMAX * N] = { 0.0 };

	int mDelayIndex = 0;

	dhdGetDeltaJointAngles(&mj[0], &mj[1], &mj[2], master);

	// joint angle initialization
	dhdGetDeltaJointAngles(&mj[0], &mj[1], &mj[2], master);

	for (int i = 0; i < N; i++) {
		mjvBack[i] = mj[i];
	}
	
	for (int i = 0; i < DELAYCONSTMAX * N; i++) {
		mjBack[i] = mj[i];	
	}

	// sync start time for control loop
	mIndex = 1;
	while (!sIndex) {

	}

	mLastTempTime = dhdGetTime();

	while (!done) {

		mTotalTime = delayConst * CYCLETIME;

		// get anglular position and velocity
		dhdGetDeltaJointAngles(&mj[0], &mj[1], &mj[2], master);
		dhdDeltaGravityJointTorques(mj[0], mj[1], mj[2], &mq[0], &mq[1], &mq[2], master);
		
		// calculate loop time
		mTempTime = dhdGetTime();
		mCycleTime = mTempTime - mLastTempTime;
		mLastTempTime = mTempTime;

		// calculate velocity		
		velocityCalculator(mv, mvBackv, mvEst, mj, mjvBack, mCycleTime);
		
		// calculate vm velocity
		waveVelocityCalculator(vmv, vmvBack, vmBack, vsDelay, mCycleTime, sTotalTime, mDelayIndex);
		
		// decode and encode wave variable
		masterWaveDecodeEncode(um, vm, vmv, vsDelay, mt, mv, sTotalTime);		
		
		// set joint torque
		dhdSetDeltaJointTorques(mt[0] + mq[0], mt[1] + mq[1], mt[2] + mq[2], master);

		// get Cartesian position
		dhdGetPosition(&mp[0], &mp[1], &mp[2], master);

		// set backward wave
		delayMimic(umDelay, umBack, um, mDelayIndex);
		delayMimic(vmDelay, vmBack, vm, mDelayIndex);
		delayMimic(mjDelay, mjBack, mj, mDelayIndex);
		if (mDelayIndex == DELAYCONSTMAX - 1) mDelayIndex = 0;
		else mDelayIndex++;

	}

	// close the connection
	//drdClose(master);
	return NULL;
}
