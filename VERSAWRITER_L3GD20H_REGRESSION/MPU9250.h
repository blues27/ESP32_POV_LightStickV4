#ifndef ___MPU9250_H____
#define ___MPU9250_H____

#include "math.h"
 
// Original Information from RM-MPU-9250A-00
// https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf

#define MPU9250_I2C_ADDR       0x69

// Register Map for Gyroscope and Accelerometer
#define MPU9250_SELF_TEST_X_GYRO    0x00    // Gyroscope Self-Test Registers
#define MPU9250_SELF_TEST_Y_GYRO    0x01                                                                          
#define MPU9250_SELF_TEST_Z_GYRO    0x02
#define MPU9250_SELF_TEST_X_ACCEL   0x0D    // Accelerometer Self-Test Registers
#define MPU9250_SELF_TEST_Y_ACCEL   0x0E    
#define MPU9250_SELF_TEST_Z_ACCEL   0x0F

#define MPU9250_XG_OFFSET_H         0x13    // Gyro Offset Registers
#define MPU9250_XG_OFFSET_L         0x14
#define MPU9250_YG_OFFSET_H         0x15
#define MPU9250_YG_OFFSET_L         0x16
#define MPU9250_ZG_OFFSET_H         0x17
#define MPU9250_ZG_OFFSET_L         0x18

#define MPU9250_SMPLRT_DIV          0x19    // Sample Rate Divider
#define MPU9250_CONFIG              0x1A    // Configuration 
#define MPU9250_GYRO_CONFIG         0x1B    // Gyroscope Configuration 
#define MPU9250_ACCEL_CONFIG        0x1C    // Accelerometer Configuration
#define MPU9250_ACCEL_CONFIG2       0x1D    // Accelerometer Configuration 2
#define MPU9250_LP_ACCEL_ODR        0x1E    // Low Power Accelerometer ODR Control
#define MPU9250_WOM_THR             0x1F    // Wake-on Motion Threshold 

#define MPU9250_FIFO_EN             0x23    // FIFO Enable
#define MPU9250_I2C_MST_CTRL        0x24    // I2C Master Control
                                            // I2C Slave 0 Control
#define MPU9250_I2C_SLV0_ADDR       0x25    
#define MPU9250_I2C_SLV0_REG        0x26
#define MPU9250_I2C_SLV0_CTRL       0x27
                                            // I2C Slave 1 Control
#define MPU9250_I2C_SLV1_ADDR       0x28
#define MPU9250_I2C_SLV1_REG        0x29
#define MPU9250_I2C_SLV1_CTRL       0x2A
                                            // I2C Slave 2 Control
#define MPU9250_I2C_SLV2_ADDR       0x2B
#define MPU9250_I2C_SLV2_REG        0x2C
#define MPU9250_I2C_SLV2_CTRL       0x2D
                                            // I2C Slave 3 Control
#define MPU9250_I2C_SLV3_ADDR       0x2E
#define MPU9250_I2C_SLV3_REG        0x2F
#define MPU9250_I2C_SLV3_CTRL       0x30
                                            // I2C Slave 4 Control
#define MPU9250_I2C_SLV4_ADDR       0x31
#define MPU9250_I2C_SLV4_REG        0x32
#define MPU9250_I2C_SLV4_DO         0x33
#define MPU9250_I2C_SLV4_CTRL       0x34
#define MPU9250_I2C_SLV4_DI         0x35

#define MPU9250_I2C_MST_STATUS      0x36    // I2C Master Status 

#define MPU9250_INT_PIN_CFG         0x37    // INT Pin/Bypass Enable Configuration 
#define MPU9250_INT_ENABLE          0x38    // Interrupt Enable
#define MPU9250_INT_STATUS          0x3A    // Interrupt Status
                                            // Accelerometer Measurements
#define MPU9250_ACCEL_XOUT_H        0x3B    
#define MPU9250_ACCEL_XOUT_L        0x3C
#define MPU9250_ACCEL_YOUT_H        0x3D
#define MPU9250_ACCEL_YOUT_L        0x3E
#define MPU9250_ACCEL_ZOUT_H        0x3F
#define MPU9250_ACCEL_ZOUT_L        0x40
                                            // Temperature Measurement
#define MPU9250_TEMP_OUT_H          0x41
#define MPU9250_TEMP_OUT_L          0x42
                                            // Gyroscope Measurements
#define MPU9250_GYRO_XOUT_H         0x43
#define MPU9250_GYRO_XOUT_L         0x44
#define MPU9250_GYRO_YOUT_H         0x45
#define MPU9250_GYRO_YOUT_L         0x46
#define MPU9250_GYRO_ZOUT_H         0x47
#define MPU9250_GYRO_ZOUT_L         0x48
                                            // External Sensor Data
#define MPU9250_EXT_SENS_DATA_00    0x49
#define MPU9250_EXT_SENS_DATA_01    0x4A
#define MPU9250_EXT_SENS_DATA_02    0x4B
#define MPU9250_EXT_SENS_DATA_03    0x4C
#define MPU9250_EXT_SENS_DATA_04    0x4D
#define MPU9250_EXT_SENS_DATA_05    0x4E
#define MPU9250_EXT_SENS_DATA_06    0x4F
#define MPU9250_EXT_SENS_DATA_07    0x50
#define MPU9250_EXT_SENS_DATA_08    0x51
#define MPU9250_EXT_SENS_DATA_09    0x52
#define MPU9250_EXT_SENS_DATA_10    0x53
#define MPU9250_EXT_SENS_DATA_11    0x54
#define MPU9250_EXT_SENS_DATA_12    0x55
#define MPU9250_EXT_SENS_DATA_13    0x56
#define MPU9250_EXT_SENS_DATA_14    0x57
#define MPU9250_EXT_SENS_DATA_15    0x58
#define MPU9250_EXT_SENS_DATA_16    0x59
#define MPU9250_EXT_SENS_DATA_17    0x5A
#define MPU9250_EXT_SENS_DATA_18    0x5B
#define MPU9250_EXT_SENS_DATA_19    0x5C
#define MPU9250_EXT_SENS_DATA_20    0x5D
#define MPU9250_EXT_SENS_DATA_21    0x5E
#define MPU9250_EXT_SENS_DATA_22    0x5F
#define MPU9250_EXT_SENS_DATA_23    0x60
                                        
#define MPU9250_I2C_SLV0_DO         0x63    // I2C Slave 0 Data Out
#define MPU9250_I2C_SLV1_DO         0x64    // I2C Slave 1 Data Out
#define MPU9250_I2C_SLV2_DO         0x65    // I2C Slave 2 Data Out
#define MPU9250_I2C_SLV3_DO         0x66    // I2C Slave 3 Data Out
#define MPU9250_I2C_MST_DELAY_CTRL  0x67    // I2C Master Delay Control

#define MPU9250_SIGNAL_PATH_RESET   0x68    // Signal Path Reset

#define MPU9250_MOT_DETECT_CTRL     0x69    // Accelerometer Interrupt Control  ACCEL_INTEL_CTRL
#define MPU9250_USER_CTRL           0x6A    // User Control
#define MPU9250_PWR_MGMT_1          0x6B    // Power Management 1 (Default Value 0x01)
#define MPU9250_PWR_MGMT_2          0x6C    // Power Management 2
                                            // FIFO Count Registers
#define MPU9250_FIFO_COUNTH         0x72    
#define MPU9250_FIFO_COUNTL         0x73
#define MPU9250_FIFO_R_W            0x74    // FIFO Read Write

#define MPU9250_WHO_AM_I            0x75    // WHO AM I (Default Value 0x71) 
                                            // Accelerometer Offset Registers
#define I_AM_MPU9250         ((uint8_t)0x71)
                                            
#define MPU9250_XA_OFFSET_H         0x77
#define MPU9250_XA_OFFSET_L         0x78
#define MPU9250_YA_OFFSET_H         0x7A
#define MPU9250_YA_OFFSET_L         0x7B
#define MPU9250_ZA_OFFSET_H         0x7D
#define MPU9250_ZA_OFFSET_L         0x7E

//Magnetometer Registers
#define AK8963_I2C_ADDR     0x0C
#define AK8963_WIA          0x00    // Device ID (Default value 0x48)
#define AK8963_INFO         0x01    // Information
#define AK8963_ST1          0x02    // Status 1
                                    // Measurement data
#define AK8963_XOUT_L       0x03  
#define AK8963_XOUT_H       0x04
#define AK8963_YOUT_L       0x05
#define AK8963_YOUT_H       0x06
#define AK8963_ZOUT_L       0x07
#define AK8963_ZOUT_H       0x08

#define AK8963_ST2          0x09    // Status 2
#define AK8963_CNTL1        0x0A    // Control1
#define AK8963_CNTL2        0x0B    // Control2
#define AK8963_RSV          0x0C    // Reserved 
#define AK8963_ASTC         0x0C    // Self-test
#define AK8963_I2CDIS       0x0F    // I2C Disable
#define AK8963_ASAX         0x10    // X-axis sensitivity adjustment value
#define AK8963_ASAY         0x11    // Y-axis sensitivity adjustment value
#define AK8963_ASAZ         0x12    // Z-axis sensitivity adjustment value

#define MPU9250_H_RESET     0x80
#define MPU9250_SLEEP       0x40
#define MPU9250_CYCLE       0x20
#define MPU9250_GYRO_STADBY 0x10
#define MPU9250_PD_PTAT     0x08
#define MPU9250_CLKSEL_STOP 0x07
#define MPU9250_CLKSEL_AUTO 0x03
#define MPU9250_CLKSEL_I20  0x00
#define MPU9250_BYPASS_EN   0x02

#define MPI9250_GFS_250     (0x00<<3)
#define MPI9250_GFS_500     (0x01<<3)
#define MPI9250_GFS_1000    (0x02<<3)
#define MPI9250_GFS_2000    (0x03<<3)

#define MPI9250_AFS_2G      (0x00<<3)
#define MPI9250_AFS_4G      (0x01<<3)
#define MPI9250_AFS_8G      (0x02<<3)
#define MPI9250_AFS_16G     (0x03<<3)

#define MPI9250_GBW_250     0x00
#define MPI9250_GBW_184     0x01
#define MPI9250_GBW_92      0x02
#define MPI9250_GBW_41      0x03
#define MPI9250_GBW_20      0x04
#define MPI9250_GBW_10      0x05
#define MPI9250_GBW_5       0x06

#define MPI9250_ABW_460     0x00
#define MPI9250_ABW_184     0x01
#define MPI9250_ABW_92      0x02
#define MPI9250_ABW_41      0x03
#define MPI9250_ABW_20      0x04
#define MPI9250_ABW_10      0x05
#define MPI9250_ABW_5       0x06
//#define MPI9250_ABW_460     0x07

#define MPU9250_WRITE       0x00
#define MPU9250_READ        0x80

// Set initial input parameters
enum MPU9250_ACCEL_FS {
    ACCEL_FS_SEL_2G = 0,
    ACCEL_FS_SEL_4G,
    ACCEL_FS_SEL_8G,
    ACCEL_FS_SEL_16G
};

enum MPU9250_GYRO_FS {
    GYRO_FS_SEL_250DPS = 0,
    GYRO_FS_SEL_500DPS,
    GYRO_FS_SEL_1000DPS,
    GYRO_FS_SEL_2000DPS
};

enum MPU9250_MAG_BIT {
    MAG_BIT_14BITS = 0, // 0.6uT/LSB
    MAG_BIT_16BITS      // 0.15uT/LSB
};

#define MPU9250_DLPF_CFG_256HZ_NOLPF2  0x00
#define MPU9250_DLPF_CFG_184HZ         0x01 // Low-pass filter Gyro 184Hz, Temp
#define MPU9250_DLPF_CFG_98HZ          0x02
#define MPU9250_DLPF_CFG_42HZ          0x03
#define MPU9250_DLPF_CFG_20HZ          0x04
#define MPU9250_DLPF_CFG_10HZ          0x05
#define MPU9250_DLPF_CFG_5HZ           0x06
#define MPU9250_DLPF_CFG_2100HZ_NOLPF  0x07

#define AK8963_MODE_PWRDOWN     0x00    // Power-down mode
#define AK8963_MODE_SNGLMES     0x01    // Single measurement mode  
#define AK8963_MODE_CONMES1     0x02    // Continuous measurement mode 1 
#define AK8963_MODE_CONMES2     0x06    // Continuous measurement mode 2
#define AK8963_MODE_EXTTRIG     0x04    // External trigger measurement mode
#define AK8963_MODE_SELFTST     0x08    // Self-test mode 
#define AK8963_MODE_FUSEROM     0x0F    // Fuse ROM access mode

#define D2D_MPU9250 (0.06097561)

#endif
