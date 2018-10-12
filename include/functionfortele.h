#ifndef _functionfortele_h
#define _functionfortele_h

int velocityCalculator(double *v, double *vBack, double *vEst, double *j, double *jBack, double time);
int waveVelocityCalculator(double *wavev, double *wavevBack, double *wavePos, double *waveInput, double time, double totalTime, int delayIndex);
int masterWaveDecodeEncode(double *u, double *v, double *wavev, double *waveInput, double *torque, double *velocity, double totalTime, double b);
int slaveWaveDecodeEncode(double *u, double *v, double *wavev, double *waveInput, double *torque, double *velocity, double *position, double *vd, double *jd, double *jdBack, double *jInput, double time, double totalTime, double b);
int delayMimic(double *varDelay, double *varBack, double *var, int delayIndex, int delayConst);

#endif
