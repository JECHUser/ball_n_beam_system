#ifndef I2C_H_
#define I2C_H_
#include <stdio.h>
#include "include.h"
#define I2C_PORT 			I2C1_BASE
//#define I2C0_ENABLE_
// setup code for all I2C modules present on the LM4F120E5QR
void i2c_Config(void);

// functions for writing/reading single bytes of data
uint8_t i2c_ReadByte(uint8_t devId, uint8_t addr);

int32_t i2c_WriteByte(uint8_t devId, uint8_t addr, uint8_t data);

// functions for writing/reading multiple bytes of data
int32_t i2c_ReadBuf(uint8_t devId, uint8_t addr, int32_t nBytes , uint8_t* pBuf );

int32_t i2c_WriteBuf(uint8_t devId, uint8_t addr, int32_t nBytes , uint8_t* pBuf);

#endif /* I2C_H_ */
