#include "Kalman.h"

void initKalman(Kalman *KalmanStruct)
{
	/* User-tuning variables */
	(*KalmanStruct).Q_angle = 0.001f;
	(*KalmanStruct).Q_bias = 0.003f;
	(*KalmanStruct).R_measure = 0.03;

	(*KalmanStruct).angle = 0.0f; // Reset
	(*KalmanStruct).bias = 0.0f;

	(*KalmanStruct).P[0][0] = 0.0f; // Assume that the bias is 0 and we know the starting angle (use setAngle),
	//the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
	(*KalmanStruct).P[0][1] = 0.0f;
	(*KalmanStruct).P[1][0] = 0.0f;
	(*KalmanStruct).P[1][1] = 0.0f;
}

// The angle should be in degrees and the rate should be in degrees per second, dt is in second
float getAngle(Kalman *KalmanStruct, float newAngle, float newRate, float dt)
{

	// "Predict" phase
	// Step 1: Project the state ahead
	(*KalmanStruct).rate= newRate - (*KalmanStruct).bias;
	(*KalmanStruct).angle += dt * (*KalmanStruct).rate; //rate is unbiased

	// Step 2: Project the error covariance matrix ahead
	//matrix A = [1 -dt; 0 1]
	(*KalmanStruct).P[0][0] += dt * (dt*(*KalmanStruct).P[1][1] - (*KalmanStruct).P[0][1] - (*KalmanStruct).P[1][0] + (*KalmanStruct).Q_angle);
	(*KalmanStruct).P[0][1] -= dt * (*KalmanStruct).P[1][1];
	(*KalmanStruct).P[1][0] -= dt * (*KalmanStruct).P[1][1];
	(*KalmanStruct).P[1][1] += dt * (*KalmanStruct).Q_bias;

	// "Innovation" phase
	// Step 3: Calculate angle and bias - Update estimate with measurement y (newAngle)
	float y = newAngle - (*KalmanStruct).angle; // Angle difference

	// Step 4: Calculate innovation covariance matrix
	float S = (*KalmanStruct).P[0][0] + (*KalmanStruct).R_measure; // Estimate error, H*P*H = P[0][0] because H = [1 0]

	// Step 5: Calculate Kalman gain
	float K[2];
	// K = P*H'*(H*P*H'+R)^-1
	K[0] = (*KalmanStruct).P[0][0] / S;
	K[1] = (*KalmanStruct).P[1][0] / S;

	// Step 6: Update the estimate of current state
	(*KalmanStruct).angle += K[0] * y;
	(*KalmanStruct).bias += K[1] * y;

	// Step 7: Update the error covariance matrix
	float P00_temp = (*KalmanStruct).P[0][0];
	float P01_temp = (*KalmanStruct).P[0][1];

	(*KalmanStruct).P[0][0] -= K[0] * P00_temp;
	(*KalmanStruct).P[0][1] -= K[0] * P01_temp;
	(*KalmanStruct).P[1][0] -= K[1] * P00_temp;
	(*KalmanStruct).P[1][1] -= K[1] * P01_temp;

	return (*KalmanStruct).angle;
}

// Used to set angle, this should be set as the starting angle
void setAngle(Kalman *KalmanStruct, float angle)
{
	(*KalmanStruct).angle=angle;
}

// Return the unbiased rate
float getRate(Kalman KalmanStruct)
{
	return KalmanStruct.rate;
}

/* These are used to tune the Kalman filter */
void setQangle(Kalman *KalmanStruct, float Q_angle)
{
	(*KalmanStruct).Q_angle=Q_angle;
}
void setQbias(Kalman *KalmanStruct, float Q_bias)
{
	(*KalmanStruct).Q_bias=Q_bias;
}
void setRmeasure(Kalman *KalmanStruct, float R_measure)
{
	(*KalmanStruct).R_measure=R_measure;
}

float getQangle(Kalman KalmanStruct)
{
	return KalmanStruct.Q_angle;
}
float getQbias(Kalman KalmanStruct)
{
	return KalmanStruct.Q_bias;
}
float getRmeasure(Kalman KalmanStruct)
{
	return KalmanStruct.R_measure;
}



