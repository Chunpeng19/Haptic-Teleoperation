#define _USE_MATH_DEFINES
#include <math.h>
#include "functionfortele.h"

#define N 3
// parameters for velocity calculator
#define PI 3.1415926
#define FREQN 200
#define zeta 0.9
#define ALPHA 1.0e-1
#define LAMDA 5.0e-4

// parameter for wave velocity 
#define BETA 1.0e-4

#define DELAYCONSTMAX 8000

#define DEFAULT_K 12.0  // 12.0


// function for calculating velocity
int velocityCalculator(double *v, double *vBack, double *vEst, double *j, double *jBack, double time) {
	double omegan = 2 * PI * FREQN;		
	double K = omegan / tan(omegan * time / 2);	
	double a0 = K * K + 4 * K * zeta * omegan + omegan * omegan;
	double a1 = 2 * omegan * omegan - 2 * K * K;
	double a2 = K * K - 4 * K * zeta * omegan + omegan * omegan;
	double b0 = omegan * omegan;
	double b1 = 2 * omegan * omegan;
	double b2 = omegan * omegan;
	
	for (int i = 0; i < N; i++) {
		vEst[i] = (j[i] - jBack[i]) / time;
		jBack[i] = j[i];
		//v[i] = (b0 * vEst[i] + b1 * vEst[N + i] + b2 * vEst[2 * N + i] - a1 * vBack[i] - a2 * vBack[N + i]) / a0;
		v[i] = vEst[i] * ALPHA + vBack[i] * (1 - ALPHA);
		vBack[N + i] = vBack[i];
		vBack[i] = v[i];
		vEst[2 * N + i]	= vEst[N + i];
		vEst[N + i] = vEst[i];
	}

	return 0;
	
}


// function for calculating wave velocity
int waveVelocityCalculator(double *wavev, double *wavevBack, double *wavePos, double *waveInput, double time, double totalTime, int delayIndex) {
	
	for (int i = 0; i < N; i++) {
		if (delayIndex == 0) wavev[i] = ((waveInput[i] - wavePos[(DELAYCONSTMAX - 1) * N + i]) / time * BETA + (1 - BETA)*wavevBack[i]) / (1 + totalTime / time * BETA);
		else wavev[i] = ((waveInput[i] - wavePos[(delayIndex - 1) * N + i]) / time * BETA + (1 - BETA)*wavevBack[i]) / (1 + totalTime / time * BETA);
		wavevBack[i] = wavev[i];
	}

	return 0;
	
}

// function for master's wave decode and encode
int masterWaveDecodeEncode(double *u, double *v, double *wavev, double *waveInput, double *torque, double *velocity, double totalTime, double b) {
	for (int i = 0; i < N; i++) {			
		v[i] = waveInput[i] - wavev[i] * totalTime;
		torque[i] = -(b * velocity[i] - sqrt(2 * b)*v[i]);
		u[i] = sqrt(2 * b)*velocity[i] - v[i];			
	}
	
	return 0;

}


// function for slave's wave decode and encode
int slaveWaveDecodeEncode(double *u, double *v, double *wavev, double *waveInput, double *torque, double *velocity, double *position, double *vd, double *jd, double *jdBack, double *jInput, double time, double totalTime, double b) {
	
	double Kp = DEFAULT_K;
	double Kv = Kp / 100;

	for (int i = 0; i < N; i++) {			
		u[i] = waveInput[i] - wavev[i] * totalTime;
		jd[i] += vd[i] * time + (jInput[i] - jdBack[i]) * LAMDA;
		vd[i] = (sqrt(2 * b)*u[i] + Kv * velocity[i] + Kp * (position[i] - jd[i])) / (Kv + b);
		jdBack[i] = jd[i];
		torque[i] = Kv * (vd[i] - velocity[i]) + Kp * (jd[i] - position[i]);
		v[i] = u[i] - sqrt(2 / b)*torque[i];		
	}
	
	return 0;

}


// function for mimicing delay
int delayMimic(double *varDelay, double *varBack, double *var, int delayIndex, int delayConst) {

	for (int i = 0; i < N; i++) {
		if (delayIndex - delayConst < 0) {
			varDelay[i] = varBack[(delayIndex + DELAYCONSTMAX - delayConst) * N + i];		
		}
		else {
			varDelay[i] = varBack[(delayIndex - delayConst) * N + i];		
		}
		varBack[delayIndex * N + i] = var[i];	
	}
	
	return 0;

}

