#ifndef _functionfortele_h
#define _functionfortele_h

/***********************/
// Parameters

#define N 3
#define CYCLETIME 0.137e-3 // 0.25e-3 for windows and 0.137 for linux
#define ENC 20300 // encoder reading corresponding to origin

#define WAVEIMPEDANCE 0.16 // from 0.01 to 1.0; 0.05 is the min value to stablize the system at no-delay case 0.16
#define MAXWAVEIMPEDANCE 0.30
#define MINWAVEIMPEDANCE 0.01
#define WAVEIMPEDANCESTEP 0.01

#define DEFAULT_K 12.0  // 12.0
#define DELAYCONST 1 // one-way delay equals to delayconst*cycletime
#define DELAYCHANGESTEP 91
#define DELAYCONSTMAX 8000

#define ALPHA 0.7e-1 // velocity filter parameter
#define BETA 1.0e-4 // wave velocity filter parameter
#define LAMDA 5.0e-4

#define PI 3.1415926
#define FREQN 3.0e2
#define ZETA 0.707

#define VELOCITYTHRESHOLD 150

/***********************/
// Functions

int velocityCalculator(double *v, double *vBack, double *vEst, double *j, double *jBack, double time);

int waveVelocityCalculator(double *wavev, double *wavevBack, double *wavePos, double *waveInput, double time, double totalTime, int delayIndex);

int masterWaveDecodeEncode(double *u, double *v, double *wavev, double *waveInput, double *torque, double *velocity, double totalTime, double b);

int slaveWaveDecodeEncode(double *u, double *v, double *wavev, double *waveInput, double *torque, double *velocity, double *position, double *vd, double *jd, double *jdBack, double *jInput, double time, double totalTime, double b);

int delayMimic(double *varDelay, double *varBack, double *var, int delayIndex, int delayConst);

#endif
