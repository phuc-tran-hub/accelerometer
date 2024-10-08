// Simple C library for the Adafruit LSM303AGR accelerometer/magnetometer.
// E.W. Hansen, 10.28.2020 for Engs 28 21W
// 
// Borrows from Adafruit_LSM303_U.h and my MS8607.h.  Accelerometer only.

// Sensor calibration constants (Datasheet, Section 2)
#define ACCEL_GRAVITY		9.8			// "one g"
#define ACCEL_RANGE			2			// g, see CTRL_REG4

// I2C address for the sensor
#define LSM303_ADDRESS_ACCEL          (0x32 >> 1)         // 0011001x

// ACCELEROMETER SECTION
// Accelerometer registers (Sec 6, 7.1)	        // DEFAULT    TYPE
#define LSM303_WHO_AM_I_A		   0x0F	  // 00110011	  r
#define LSM303_CTRL_REG1_A         0x20   // 00000111   rw
#define LSM303_CTRL_REG2_A         0x21   // 00000000   rw
#define LSM303_CTRL_REG3_A         0x22   // 00000000   rw
#define LSM303_CTRL_REG4_A         0x23   // 00000000   rw
#define LSM303_CTRL_REG5_A         0x24   // 00000000   rw
#define LSM303_CTRL_REG6_A         0x25   // 00000000   rw
#define LSM303_REFERENCE_A         0x26   // 00000000   r
#define LSM303_STATUS_REG_A        0x27   // 00000000   r
#define LSM303_OUT_X_L_A           0x28
#define LSM303_OUT_X_H_A           0x29
#define LSM303_OUT_Y_L_A           0x2A
#define LSM303_OUT_Y_H_A           0x2B
#define LSM303_OUT_Z_L_A           0x2C
#define LSM303_OUT_Z_H_A           0x2D
#define LSM303_FIFO_CTRL_REG_A     0x2E
#define LSM303_FIFO_SRC_REG_A      0x2F
#define LSM303_INT1_CFG_A          0x30
#define LSM303_INT1_SRC_A          0x31
#define LSM303_INT1_THS_A          0x32
#define LSM303_INT1_DURATION_A     0x33
#define LSM303_INT2_CFG_A          0x34
#define LSM303_INT2_SRC_A          0x35
#define LSM303_INT2_THS_A          0x36
#define LSM303_INT2_DURATION_A     0x37
#define LSM303_CLICK_CFG_A         0x38
#define LSM303_CLICK_SRC_A         0x39
#define LSM303_CLICK_THS_A         0x3A
#define LSM303_TIME_LIMIT_A        0x3B
#define LSM303_TIME_LATENCY_A      0x3C
#define LSM303_TIME_WINDOW_A       0x3D

// Because each sensor outputs an (x, y, z) triple for acceleration field and
// magnetic field, it makes sense to put these in structures.
typedef struct{
        int16_t x;
        int16_t y;
        int16_t z;
    } lsm303AccelData_s;
    
// C function prototypes for accelerometer driver
// These are low-level functions that interact with the registers via I2C,
// and are called by higher-level functions used by the top
// level application.

// The accelerometer has several gain and update rate settings.  These are described
// in Section 7.1 of the datasheet and are set up in the AccelInit function.
uint8_t lsm303_AccelInit();			

// The AccelRegisterWrite function should take a register address and a command byte
// to be written into the register.
void lsm303_AccelRegisterWrite(uint8_t RegisterAddress, uint8_t data);		

// The AccelRegisterRead function should take a register address and return a data byte 
// read from the register.
uint8_t lsm303_AccelRegisterRead(uint8_t RegisterAddress);

// Here is the higher-level function to read the acceleration vector.  Output struct
// contains raw (unscaled) acceleration values.
void lsm303_AccelReadRaw(lsm303AccelData_s * const result);


