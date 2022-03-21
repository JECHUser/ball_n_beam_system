#include "myTimer.h"
#include <math.h>
extern int16_t AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ;

double roll, pitch, yaw=0;
extern double gyroRoll, gyroPitch, gyroYaw;
extern double compRoll, compPitch, compYaw;
extern double kalRoll, kalPitch, kalYaw = 0;
extern Kalman RollKalman, PitchKalman;
extern bool KalmanStarted;
float test=20.0f;

double count = 0;
double err_sumsquare1, err_sumsquare2, err_sumsquare3;
double rms1, rms2, rms3;

void CalculateAngleRMS(double setvalue1, double getvalue1, double setvalue2, double getvalue2, double setvalue3, double getvalue3)
{
	count++;
	err_sumsquare1 += fabs(setvalue1-getvalue1)*fabs(setvalue1-getvalue1);
	rms1 = sqrt(err_sumsquare1/count);
	err_sumsquare2 += fabs(setvalue2-getvalue2)*fabs(setvalue2-getvalue2);
	rms2 = sqrt(err_sumsquare2/count);
	err_sumsquare3 += fabs(setvalue3-getvalue3)*fabs(setvalue3-getvalue3);
	rms3 = sqrt(err_sumsquare3/count);
}

void ConfigIntervalTimer(uint32_t TimerIntervalms)
{

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
	TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER5_BASE, TIMER_A, (SYSTEM_CLOCK / 1000) * TimerIntervalms);
	TimerIntRegister(TIMER5_BASE, TIMER_A, &TimerInterruptHandler);
	IntEnable(INT_TIMER5A);
	TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER5_BASE, TIMER_A);

}
void ConfigUpdateTimer(uint32_t TimerIntervalms)
{

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
	TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER4_BASE, TIMER_A, (SYSTEM_CLOCK / 1000) * TimerIntervalms);
	TimerIntRegister(TIMER4_BASE, TIMER_A, &TimerUpdateHandler);
	IntEnable(INT_TIMER4B);
	TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER4_BASE, TIMER_A);
}

void TimerInterruptHandler(void)
{

	TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	if (KalmanStarted)
	{
		mpu6050_Read_All(&AccelX, &AccelY, &AccelZ, &GyroX, &GyroY, &GyroZ);
		roll  = atan2(AccelY, AccelZ) * RAD_TO_DEG;
		pitch = atan(-AccelX / sqrt(AccelY * AccelY + AccelZ * AccelZ)) * RAD_TO_DEG;
		double gyroXrate = GyroX / 131.0; // Convert to deg/s
		double gyroYrate = GyroY / 131.0; // Convert to deg/s

		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((roll < -90 && kalRoll > 90) || (roll > 90 && kalRoll < -90))
		{
			setAngle(&RollKalman,roll);
			compRoll = roll;
			kalRoll = roll;
			gyroRoll = roll;
		}
		else
			kalRoll = getAngle(&RollKalman, roll, gyroXrate, KALMAN_PERIOD); // Calculate the angle using a Kalman filter

		if (abs(kalRoll) > 90)
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading

		kalPitch = getAngle(&PitchKalman, pitch, gyroYrate, KALMAN_PERIOD);

		compRoll = COMPLIMENT_COEFFICIENT * (compRoll + gyroXrate * KALMAN_PERIOD) + (1-COMPLIMENT_COEFFICIENT) * roll; // Calculate the angle using a Complimentary filter
		compPitch = COMPLIMENT_COEFFICIENT * (compPitch + gyroYrate * KALMAN_PERIOD) + (1-COMPLIMENT_COEFFICIENT) * pitch;
		compYaw = 0;

		// Reset the gyro angle when it has drifted too much
		if (compRoll < -180 || compRoll > 180)
			compRoll = kalRoll;
		if (compPitch < -180 || compPitch > 180)
			compPitch = kalPitch;

	}
}

void TimerUpdateHandler(void)
{
	TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	//		Data Conditioning for Processing:
	//		data packet:
	//		"roll,pitch,yaw!", x 1000 (/1000 at Processing)
	//		Ex: 0.16234,0.20715,15.6253 --> 1620,2070,15625!

	if (KalmanStarted)
	{
	int16_t calib = 1000;
	#ifdef UART_GUI_BLUETOOTH_BASE
	{
		#ifdef KALMAN_FILTERED_DATA
			UARTprintf("Kalman  Roll: ");
			int temp = kalRoll * calib;
			UARTPutn(UART1_BASE, temp);
			UARTprintf("    Pitch: ");
			temp = kalPitch * calib;
			UARTPutn(UART1_BASE, temp);
			UARTprintf("    Yaw: ");
			temp = kalYaw * calib;
			UARTPutn(UART1_BASE, temp);
			UARTprintf("!\n\r");
		#endif
		#ifdef COMPLIMENT_FILTERED_DATA
			UARTprintf("Compliment  Roll: ");
			int temp = compRoll * calib;
			UARTPutn(UART1_BASE, temp);
			UARTprintf("    Pitch: ");
			temp = compPitch * calib;
			UARTPutn(UART1_BASE, temp);
			UARTprintf("    Yaw: ");
			temp = compYaw * calib;
			UARTPutn(UART1_BASE, temp);
			UARTprintf("!\n\r");
		#endif
		#ifdef RAW_DATA
			UARTprintf("Raw  Roll: ");
			int temp = roll * calib;
			UARTPutn(UART1_BASE, temp);
			UARTprintf("    Pitch: ");
			temp = pitch * calib;
			UARTPutn(UART1_BASE, temp);
			UARTprintf("    Yaw: ");
			temp = yaw * calib;
			UARTPutn(UART1_BASE, temp);
			UARTprintf("!\n\r");
		#endif
	}

	#else
	{
		#ifdef KALMAN_FILTERED_DATA
			UARTprintf("R:");
			int temp = kalRoll * calib;
			UARTPutn(UART0_BASE, temp);
			UARTprintf(" P:");
			temp = kalPitch * calib;
			UARTPutn(UART0_BASE, temp);
			UARTprintf(" Y:");
			temp = kalYaw * calib;
			UARTPutn(UART0_BASE, temp);
			UARTprintf("\n\r");
		#endif
		#ifdef COMPLIMENT_FILTERED_DATA
			UARTprintf("R:");
			int temp = compRoll * calib;
			UARTPutn(UART0_BASE, temp);
			UARTprintf(" P:");
			temp = compPitch * calib;
			UARTPutn(UART0_BASE, temp);
			UARTprintf(" Y:");
			temp = compYaw * calib;
			UARTPutn(UART0_BASE, temp);
			UARTprintf("\n\r");
		#endif
		#ifdef RAW_DATA
			UARTprintf("R:");
			int temp = roll * calib;
			UARTPutn(UART0_BASE, temp);
			UARTprintf(" P:");
			temp = pitch * calib;
			UARTPutn(UART0_BASE, temp);
			UARTprintf(" Y:");
			temp = yaw * calib;
			UARTPutn(UART0_BASE, temp);
			UARTprintf("\n\r");
		#endif
	}
	#endif
	}
	#ifdef CHECK_E_RMS
	{
		int calib = 1000;
		CalculateAngleRMS(ROLL_SET, kalRoll+0.553, PITCH_SET, kalPitch+1.2, YAW_SET, kalYaw);
		int temp2 = rms1 * calib;
		UARTPutn(UART0_BASE, temp2);
		UARTprintf(" ");
		temp2 = rms2 * calib;
		UARTPutn(UART0_BASE, temp2);
		UARTprintf(" ");
		temp2 = rms3 * calib;
		UARTPutn(UART0_BASE, temp2);
		UARTprintf("\n\r");
	}
	#endif
}


