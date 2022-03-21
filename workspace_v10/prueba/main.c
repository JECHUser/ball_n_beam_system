#include "include.h"

/******************************************************************************
 * GLOBAL VARIABLES
 *****************************************************************************/
int16_t AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ;


float gyroRoll, gyroPitch, gyroYaw;
float compRoll, compPitch, compYaw;
double kalRoll, kalPitch;
Kalman RollKalman, PitchKalman;
bool KalmanStarted = false;

/******************************************************************************
 * MAIN
 *****************************************************************************/
void main(void)
{
	//Configuring system clock at 80MHz
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |SYSCTL_XTAL_16MHZ);
	// Enable the floating-point unit.
	FPUEnable();
	// Configure the floating-point unit to perform lazy stacking of the floating-point state.
	FPULazyStackingEnable();

	i2c_Config();

	initMPU6050();

#ifdef UART_GUI_BLUETOOTH_BASE
	ConfigUART1();
#else
	ConfigUART0();
#endif

	ConfigIntervalTimer(10);
	ConfigUpdateTimer(504);
	IntMasterEnable();

	initKalman(&PitchKalman);
	initKalman(&RollKalman);
	mpu6050_Read_All(&AccelX, &AccelY, &AccelZ,&GyroX, &GyroY, &GyroZ);

	double roll  = atan2(AccelY, AccelZ) * RAD_TO_DEG;
	double pitch = atan(-AccelX / sqrt(AccelY * AccelY + AccelZ * AccelZ)) * RAD_TO_DEG;

	// Set starting angle
	setAngle(&RollKalman,roll);
	setAngle(&PitchKalman,pitch);
	gyroRoll = roll;
	gyroPitch = pitch;
	compPitch = roll;
	compPitch = pitch;

	//Start the kalman filters
	KalmanStarted=true;

	while(1)
	{

	}
}

