/**************************************************************************
 * Simple C library for the Adafruit LSM303AGR accelerometer.
 * E.W. Hansen, Engs 28 22W
 * Accelerometer only, no magnetometer
 */

#include "i2c.h"
#include "lsm303agr.h"

/**************************************************************************
 * Write one byte to an accelerometer register (Datasheet, Table 20)
 */
void lsm303_AccelRegisterWrite(uint8_t RegisterAddress, uint8_t data) {	
	i2cStart();
	i2cSend(LSM303_ADDRESS_ACCEL << 1);  // address with write
	i2cSend(RegisterAddress);						 // register (subaddress)
	i2cSend(data);											 // send data
	i2cStop();
}

/**************************************************************************
 * Read one byte from an accelerometer register (Datasheet, Table 22)
 */
uint8_t lsm303_AccelRegisterRead(uint8_t RegisterAddress) {		
	i2cStart();
	i2cSend(LSM303_ADDRESS_ACCEL << 1);  // address with write
	i2cSend(RegisterAddress);			 // register (subaddress)
	i2cStart();				 			 			// repeated start
	i2cSend(LSM303_ADDRESS_ACCEL << 1 | 0x01);  // address with read
	uint8_t data = i2cReadNoAck();
	i2cStop();
  	
  return data;
}

/**************************************************************************
 * Initialize the sensor
 * There are 8 control registers (Datasheet, Section 8.6), but only need the
 * first one for basic operation.  May generally accept defaults on the others, 
 * e.g., full-scale range is set in Register 4.
 */
uint8_t lsm303_AccelInit() {
		// Check the WHOAMI register
  	if (lsm303_AccelRegisterRead(LSM303_WHO_AM_I_A) != 0x33)
  	{
    	return 0;		// failure
  	}
  	else {
		/******* Control register 1 (Datasheet 8.6) *******/
		uint8_t ctrl_reg1 = 0b01010111;	
		lsm303_AccelRegisterWrite(LSM303_CTRL_REG1_A, ctrl_reg1);
  
		/******* Control register 4 (Datasheet 8.9) *******/
		uint8_t ctrl_reg4 = 0b00000000;
		lsm303_AccelRegisterWrite(LSM303_CTRL_REG4_A, ctrl_reg4);	
		
		return 1;			// success
  	}
}

/**************************************************************************
 * Read acceleration data from the sensor
 * Pass a pointer to a structure to hold the three (int16_t) values.
 * The accelerometer is repeatedly sampling, I just have to grab the data.
 *
 * The output is a 1.15 twos complement fraction (of the full scale), left-justified.  
 * The three precision modes (high, normal, low) have 12, 10, and 8 nonzero bits, 
 * respectively. 
 * Multiply by full-scale range (2, 4, 8, or 16 g) to get acceleration.   
 * Multiply by 1000 to convert g to milli-g.
 * Shift right by 15 bits to drop the fractional part and keep the whole milli-g,
 * or make the result a float and keep the fractional part.
 *
 * When the chip is mounted horizontally, the Z component of gravitational
 * acceleration is positive upward.  Roll the circuit board over and the sign of the
 * acceleration flips to negative.  
 */
 
void lsm303_AccelReadRaw(lsm303AccelData_s * const result) {
	// const int16_t full_range = 2000;		// 2 g
	// const uint16_t bits = 15;				// 1.15 data format
	
 	int16_t accel_raw;
 
 	accel_raw = (lsm303_AccelRegisterRead(LSM303_OUT_X_H_A) << 8) 
 			   | lsm303_AccelRegisterRead(LSM303_OUT_X_L_A);
 	result->x = accel_raw;
    // result->x = (int16_t) ( ((int32_t) accel_raw * full_range) >> bits );
 	
 	accel_raw = (lsm303_AccelRegisterRead(LSM303_OUT_Y_H_A) << 8) 
 			   | lsm303_AccelRegisterRead(LSM303_OUT_Y_L_A);
 	result->y = accel_raw;
 	// result->y = (int16_t) ( ((int32_t) accel_raw * full_range) >> bits );
   
 	accel_raw = (lsm303_AccelRegisterRead(LSM303_OUT_Z_H_A) << 8) 
 			   | lsm303_AccelRegisterRead(LSM303_OUT_Z_L_A);
 	result->z = accel_raw;
 	// result->z = (int16_t) ( ((int32_t) accel_raw * full_range) >> bits );
 }


