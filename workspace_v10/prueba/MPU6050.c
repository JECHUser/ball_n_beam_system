/*
 * MPU6050.c
 *
 *  Created on: Oct 18, 2014
 *      Author: CKDTCNTT (cooperated with Mr Tom)
 */
#include "HW_MPU6050.h"
#include "include.h"
#define MPU6050_GYRO_FS 		MPU6050_GYRO_FS_250
#define MPU6050_GYRO_SENS		131									//GY_FS_250
#define MPU6050_ACCEL_FS		MPU6050_ACCEL_FS_2G
#define MPU6050_ACCEL_SENS		16384								//ACCEL_FS_2G



void i2cWriteData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length)
{
    I2CMasterSlaveAddrSet(I2C_PORT, addr, false); // Set to write mode

    I2CMasterDataPut(I2C_PORT, regAddr); // Place address into data register                        // The first to be sent
    I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_START); // Send start condition
    while (I2CMasterBusy(I2C_PORT)); // Wait until transfer is done
    uint8_t i = 0;
    for (i = 0; i < length - 1; i++) {                                                              // Just send n-1 data after
        I2CMasterDataPut(I2C_PORT, data[i]); // Place data into data register
        I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_CONT); // Send continues condition
        while (I2CMasterBusy(I2C_PORT)); // Wait until transfer is done
    }

    I2CMasterDataPut(I2C_PORT, data[length - 1]); // Place data into data register                  // The last to be sent
    I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_FINISH); // Send finish condition
    while (I2CMasterBusy(I2C_PORT)); // Wait until transfer is done
}
void i2cWrite(uint8_t addr, uint8_t regAddr, uint8_t data)
{
    i2cWriteData(addr, regAddr, &data, 1);
}
uint8_t i2cRead(uint8_t addr, uint8_t regAddr)
{
    I2CMasterSlaveAddrSet(I2C_PORT, addr, false);                                 // Set to write mode

    I2CMasterDataPut(I2C_PORT, regAddr); // Place address into data register
    I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_SINGLE_SEND); // Send data - just the regAddr
    while (I2CMasterBusy(I2C_PORT)); // Wait until transfer is done

    I2CMasterSlaveAddrSet(I2C_PORT, addr, true);                                  // Set to read mode

    I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_SINGLE_RECEIVE); // Tell master to read data
    while (I2CMasterBusy(I2C_PORT)); // Wait until transfer is done
    return I2CMasterDataGet(I2C_PORT); // Read data
}

void i2cReadData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length)
{
    I2CMasterSlaveAddrSet(I2C_PORT, addr, false); // Set to write mode

    I2CMasterDataPut(I2C_PORT, regAddr); // Place address into data register
    I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_SINGLE_SEND); // Send data
    while (I2CMasterBusy(I2C_PORT)); // Wait until transfer is done

    I2CMasterSlaveAddrSet(I2C_PORT, addr, true); // Set to read mode

    I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_RECEIVE_START); // Send start condition
    while (I2CMasterBusy(I2C_PORT)); // Wait until transfer is done
    data[0] = I2CMasterDataGet(I2C_PORT); // Place data into data register
    uint8_t i = 1;
    for (i = 1; i < length - 1; i++) {
        I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_RECEIVE_CONT); // Send continues condition
        while (I2CMasterBusy(I2C_PORT)); // Wait until transfer is done
        data[i] = I2CMasterDataGet(I2C_PORT); // Place data into data register
    }

    I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_RECEIVE_FINISH); // Send finish condition
    while (I2CMasterBusy(I2C_PORT)); // Wait until transfer is done
    data[length - 1] = I2CMasterDataGet(I2C_PORT); // Place data into data register
}

void initMPU6050(void)
{
	uint8_t i2cBuffer[5]; // Buffer for I2C data
	i2cBuffer[0] = i2cRead(MPU6050_ADDRESS, MPU6050_WHO_AM_I); // Place address into data register WHO_AM_I
	i2cWrite(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, (1 << 7)); //DEVICE _RESET[7], SLEEP[6], CYCLE[5], TEMP_DIS[3], CLKSEL[2:0]
															//Reset device, this resets all internal registers to their default values
	SysCtlDelay(SysCtlClockGet()/100);
	while (i2cRead(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1) & (1 << 7))
	{
	// Wait for the bit to clear
	};
	SysCtlDelay(SysCtlClockGet()/100);
	i2cWrite(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, (1 << 3) | (1 << 0)); // Disable sleep mode, disable temperature sensor and use PLL as clock reference

	i2cBuffer[0] = 0; 		// Set the sample rate to 1kHz - 1kHz/(1+0) = 1kHz
	i2cBuffer[1] = 0x03; 	// Disable FSYNC and set 41 Hz Gyro filtering, 1 KHz sampling
							// why 41Hz visit http://forum.taulabs.org/viewtopic.php?f=17&t=305
	i2cBuffer[2] = 3 << 3; 	// Set Gyro Full Scale Range to +-2000deg/s
	i2cBuffer[3] = 2 << 3; 	// Set Accelerometer Full Scale Range to +-8g
	i2cBuffer[4] = 0x03; 	// 41 Hz Acc filtering
	i2cWriteData(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, i2cBuffer, 5); // Write to all 5 regs starting from SMPLRT_DIV
	/* Enable Raw Data Ready Interrupt on INT pin */
	i2cBuffer[0] = (1 << 5) | (1 << 4); // Enable LATCH_INT_EN and INT_ANYRD_2CLEAR
	// When this bit is equal to 1, the INT pin is held HIGH until the interrupt is cleared
	// When this bit is equal to 1, interrupt status is cleared if any READ operation is performed
	i2cBuffer[1] = (1 << 0);    //(the next registor from PIN_CFG - Interrupt Enable)
	//Enable RAW_RDY_EN  - When set to 1, Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin
	i2cWriteData(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, i2cBuffer, 2); // Write to both registers at once
}
//*****************************************************************************************************************
//
//*****************************************************************************************************************

void mpu6050_Config()
{
	uint8_t status;

	// Reset MPU6050
	i2c_WriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x01 << MPU6050_PWR1_DEVICE_RESET_BIT);  //PWR Management 1    -- DEVICE_RESET 1
	while(1)
	{
		status=i2c_ReadByte(MPU6050_ADDRESS,MPU6050_PWR_MGMT_1);
		if(status == 0x40)
		{
			break;          //dont do anything at sleep mode 0x40
		}
		//delay ...........
		SysCtlDelay(SYSTEM_CLOCK/3000);
	}

	// Test MPU6050
	status=i2c_ReadByte(MPU6050_ADDRESS,MPU6050_WHO_AM_I);
	if(status == MPU6050_ADDRESS)
	{
	}
	else
	{
		while(1);
	}

	// Configure Gyroscope
	// SMPLRT_DIV -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	i2c_WriteByte(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, DISABLE_REG);

	//CONFIG    -- EXT_SYNC_SET 0 (disable input pin for data sync) ;
	//default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
	i2c_WriteByte(MPU6050_ADDRESS, MPU6050_CONFIG, MPU6050_DLPF_BW_5);

	//GYRO_CONFIG   -- FS_SEL = 0: Full scale set to 250 deg/sec
	i2c_WriteByte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS << MPU6050_GCONFIG_FS_SEL_BIT);

	// Configure Accelerometer
	//ACCEL_CONFIG  -- AFS_SEL=0 (Full Scale = +/-2G)  ;
	//ACCELL_HPF=0
	i2c_WriteByte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS << MPU6050_ACONFIG_AFS_SEL_BIT);

	//TurnOff SleepMode
	//i2c_WriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x01 << MPU6050_PWR1_TEMP_DIS_BIT | MPU6050_CLOCK_INTERNAL); //PWR_MGMT_1
	i2c_WriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, MPU6050_CLOCK_INTERNAL); //PWR_MGMT_1

	//Enable bypass for extenal configuration
	//USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=0 (I2C bypass mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ;
	//SIG_COND_RESET=0
	//i2c_WriteByte(MPU6050_ADDRESS, MPU6050_USER_CTRL, DISABLE_REG);

	//INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ;
	//FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
	//i2c_WriteByte(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, 0x01 << MPU6050_INTCFG_I2C_BYPASS_EN_BIT);
}
//
//******************************************************************************************************
void mpu6050_Read_GyroXYZ(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t g[6];
	i2c_ReadBuf(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, 6, g);
	*x = ((int16_t)(((uint16_t)g[0]<<8) | (uint16_t)g[1]));
	*y = ((int16_t)(((uint16_t)g[2]<<8) | (uint16_t)g[3]));
	*y = ((int16_t)(((uint16_t)g[4]<<8) | (uint16_t)g[5]));
}

//******************************************************************************************************
void mpu6050_Read_AccelXYZ(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t a[6];
	i2c_ReadBuf(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 6, a);
	*x = (int16_t)(((uint16_t)a[0]<<8) | (uint16_t)a[1]);
	*y = (int16_t)(((uint16_t)a[2]<<8) | (uint16_t)a[3]);
	*z = (int16_t)(((uint16_t)a[4]<<8) | (uint16_t)a[5]);

}
//
//*******************************************************************************************************
void mpu6050_Read_Temp(int16_t *temp)
{
	uint8_t t[2];
	int16_t temp_out;
	i2c_ReadBuf(MPU6050_ADDRESS, MPU6050_TEMP_OUT_H, 2, t);
	temp_out=((int16_t)(((uint16_t)t[0]<<8) | (uint16_t)t[1]));
	*temp = temp_out/340+36.53;
}
void mpu6050_Read_All(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,
		int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
	uint8_t buf[14];
	i2cReadData(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, buf, 14); // Note that we can't write directly into MPU6050_t, because of endian conflict. So it has to be done manually

 	*accel_x = (buf[0] << 8) | buf[1];
 	*accel_y = (buf[2] << 8) | buf[3];
 	*accel_z = (buf[4] << 8) | buf[5];

 	*gyro_x = (buf[8] << 8) | buf[9];
 	*gyro_y = (buf[10] << 8) | buf[11];
 	*gyro_z = (buf[12] << 8) | buf[13];
 	SysCtlDelay(2);
}
