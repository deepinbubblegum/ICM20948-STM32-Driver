/*
 * ICM20948.h
 *
 * Created on: Nov 13, 2025
 * Author: chaiwit
 */

#ifndef SRC_ICM20948_H_
#define SRC_ICM20948_H_

#include "main.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// clang-format off
#define GYRO_SENSITIVITY    16.4f   // ตรงกับ ±2000dps
#define ACCEL_SENSITIVITY   8192.0f // ตรงกับ ±4g
#define MAG_SENSITIVITY     0.15f   // ค่าคงที่ของ AK09916

#define READ                                0x80
#define WRITE                               0x00
#define ICM_20948_WHO_AM_I_VAL              0xEA 
#define REG_BANK_SEL                        0x7F

//==============================================================================
// USER BANK 0 Registers
//==============================================================================
#define REG_B0_WHO_AM_I						0x00		
#define REG_B0_USER_CTRL					0x03
#define REG_B0_LP_CONFIG					0x05
#define REG_B0_PWR_MGMT_1					0x06
#define REG_B0_PWR_MGMT_2					0x07
#define REG_B0_INT_PIN_CFG					0x0F		
#define REG_B0_INT_ENABLE					0x10
#define REG_B0_INT_ENABLE_1					0x11
#define REG_B0_INT_ENABLE_2					0x12
#define REG_B0_INT_ENABLE_3					0x13
#define REG_B0_I2C_MST_STATUS				0x17		
#define REG_B0_INT_STATUS					0x19		
#define REG_B0_INT_STATUS_1					0x1A
#define REG_B0_INT_STATUS_2					0x1B
#define REG_B0_INT_STATUS_3					0x1C
#define REG_B0_DELAY_TIMEH					0x28
#define REG_B0_DELAY_TIMEL					0x29
#define REG_B0_ACCEL_XOUT_H					0x2D		
#define REG_B0_ACCEL_XOUT_L					0x2E		
#define REG_B0_ACCEL_YOUT_H					0x2F		
#define REG_B0_ACCEL_YOUT_L					0x30		
#define REG_B0_ACCEL_ZOUT_H					0x31		
#define REG_B0_ACCEL_ZOUT_L					0x32	
#define REG_B0_GYRO_XOUT_H					0x33	
#define REG_B0_GYRO_XOUT_L					0x34
#define REG_B0_GYRO_YOUT_H					0x35
#define REG_B0_GYRO_YOUT_L					0x36
#define REG_B0_GYRO_ZOUT_H					0x37
#define REG_B0_GYRO_ZOUT_L					0x38
#define REG_B0_TEMP_OUT_H					0x39		
#define REG_B0_TEMP_OUT_L					0x3A
#define REG_B0_EXT_SLV_SENS_DATA_00			0x3B
#define REG_B0_EXT_SLV_SENS_DATA_01			0x3C
#define REG_B0_EXT_SLV_SENS_DATA_02			0x3D
#define REG_B0_EXT_SLV_SENS_DATA_03			0x3E
#define REG_B0_EXT_SLV_SENS_DATA_04			0x3F
#define REG_B0_EXT_SLV_SENS_DATA_05			0x40
#define REG_B0_EXT_SLV_SENS_DATA_06			0x41
#define REG_B0_EXT_SLV_SENS_DATA_07			0x42
#define REG_B0_EXT_SLV_SENS_DATA_08			0x43
#define REG_B0_EXT_SLV_SENS_DATA_09			0x44
#define REG_B0_EXT_SLV_SENS_DATA_10			0x45
#define REG_B0_EXT_SLV_SENS_DATA_11			0x46
#define REG_B0_EXT_SLV_SENS_DATA_12			0x47
#define REG_B0_EXT_SLV_SENS_DATA_13			0x48
#define REG_B0_EXT_SLV_SENS_DATA_14			0x49
#define REG_B0_EXT_SLV_SENS_DATA_15			0x4A
#define REG_B0_EXT_SLV_SENS_DATA_16			0x4B
#define REG_B0_EXT_SLV_SENS_DATA_17			0x4C
#define REG_B0_EXT_SLV_SENS_DATA_18			0x4D
#define REG_B0_EXT_SLV_SENS_DATA_19			0x4E
#define REG_B0_EXT_SLV_SENS_DATA_20			0x4F
#define REG_B0_EXT_SLV_SENS_DATA_21			0x50
#define REG_B0_EXT_SLV_SENS_DATA_22			0x51
#define REG_B0_EXT_SLV_SENS_DATA_23			0x52
#define REG_B0_FIFO_EN_1					0x66	
#define REG_B0_FIFO_EN_2					0x67
#define REG_B0_FIFO_RST						0x68
#define REG_B0_FIFO_MODE					0x69
#define REG_B0_FIFO_COUNTH					0X70
#define REG_B0_FIFO_COUNTL					0X71
#define REG_B0_FIFO_R_W						0x72
#define REG_B0_DATA_RDY_STATUS				0x74
#define REG_B0_FIFO_CFG						0x76

//==============================================================================
// USER BANK 1 Registers
//==============================================================================
#define REG_B1_SELF_TEST_X_GYRO				0x02	
#define REG_B1_SELF_TEST_Y_GYRO				0x03
#define REG_B1_SELF_TEST_Z_GYRO				0x04
#define REG_B1_SELF_TEST_X_ACCEL			0x0E	
#define REG_B1_SELF_TEST_Y_ACCEL			0x0F
#define REG_B1_SELF_TEST_Z_ACCEL			0x10
#define REG_B1_XA_OFFS_H					0x14	
#define REG_B1_XA_OFFS_L					0x15
#define REG_B1_YA_OFFS_H					0x17
#define REG_B1_YA_OFFS_L					0x18
#define REG_B1_ZA_OFFS_H					0x1A
#define REG_B1_ZA_OFFS_L					0x1B
#define REG_B1_TIMEBASE_CORRECTION_PLL		0x28

//==============================================================================
// USER BANK 2 Registers
//==============================================================================
#define REG_B2_GYRO_SMPLRT_DIV				0x00	
#define REG_B2_GYRO_CONFIG_1				0x01	
#define REG_B2_GYRO_CONFIG_2				0x02
#define REG_B2_XG_OFFS_USRH					0x03	
#define REG_B2_XG_OFFS_USRL 				0x04
#define REG_B2_YG_OFFS_USRH					0x05
#define REG_B2_YG_OFFS_USRL					0x06
#define REG_B2_ZG_OFFS_USRH					0x07
#define REG_B2_ZG_OFFS_USRL					0x08
#define REG_B2_ODR_ALIGN_EN					0x09	
#define REG_B2_ACCEL_SMPLRT_DIV_1			0x10	
#define REG_B2_ACCEL_SMPLRT_DIV_2			0x11		
#define REG_B2_ACCEL_INTEL_CTRL				0x12		
#define REG_B2_ACCEL_WOM_THR				0x13
#define REG_B2_ACCEL_CONFIG					0x14
#define REG_B2_ACCEL_CONFIG_2				0x15
#define REG_B2_FSYNC_CONFIG					0x52
#define REG_B2_TEMP_CONFIG					0x53
#define REG_B2_MOD_CTRL_USR					0X54

//==============================================================================
// USER BANK 3 Registers
//==============================================================================
#define REG_B3_I2C_MST_ODR_CONFIG			0x00
#define REG_B3_I2C_MST_CTRL					0x01
#define REG_B3_I2C_MST_DELAY_CTRL			0x02	
#define REG_B3_I2C_SLV0_ADDR				0x03
#define REG_B3_I2C_SLV0_REG					0x04		
#define REG_B3_I2C_SLV0_CTRL				0x05
#define REG_B3_I2C_SLV0_DO					0x06
#define REG_B3_I2C_SLV1_ADDR				0x07		
#define REG_B3_I2C_SLV1_REG					0x08		
#define REG_B3_I2C_SLV1_CTRL				0x09
#define REG_B3_I2C_SLV1_DO					0x0A
#define REG_B3_I2C_SLV2_ADDR				0x0B		
#define REG_B3_I2C_SLV2_REG					0x0C		
#define REG_B3_I2C_SLV2_CTRL				0x0D
#define REG_B3_I2C_SLV2_DO					0x0E
#define REG_B3_I2C_SLV3_ADDR				0x0F		
#define REG_B3_I2C_SLV3_REG					0x10		
#define REG_B3_I2C_SLV3_CTRL				0x11
#define REG_B3_I2C_SLV3_DO					0x12
#define REG_B3_I2C_SLV4_ADDR				0x13	
#define REG_B3_I2C_SLV4_REG					0x14		
#define REG_B3_I2C_SLV4_CTRL				0x15
#define REG_B3_I2C_SLV4_DO					0x16
#define REG_B3_I2C_SLV4_DI					0x17

//==============================================================================
// AK09916 (Magnetometer) Registers
//==============================================================================
#define REG_AK09916_ID						0x09
#define REG_MAG_SLAVE_ADDR                  0x0C
#define REG_MAG_WIA2						0x01
#define REG_MAG_ST1							0x10
#define REG_MAG_HXL							0x11
#define REG_MAG_HXH							0x12
#define REG_MAG_HYL							0x13
#define REG_MAG_HYH							0x14
#define REG_MAG_HZL							0x15
#define REG_MAG_HZH							0x16
#define REG_MAG_ST2							0x18
#define REG_MAG_CNTL2						0x31
#define REG_MAG_CNTL3						0x32
#define REG_MAG_TS1							0x33
#define REG_MAG_TS2							0x34

//==============================================================================
// GYRO_CONFIG_1
//==============================================================================
#define GYRO_CONFIG_1(DLPF, FS, FCHOICE) ( \
    ((DLPF)     << 3) | \
    ((FS)       << 1) | \
    ((FCHOICE)  << 0)   \
)
typedef enum
{
    GYRO_DLPFCFG_196_6Hz = 0x00,
    GYRO_DLPFCFG_151_8Hz = 0x01,
    GYRO_DLPFCFG_119_5Hz = 0x02,
    GYRO_DLPFCFG_51_2Hz  = 0x03,
    GYRO_DLPFCFG_23_9Hz  = 0x04,
    GYRO_DLPFCFG_11_6Hz  = 0x05,
    GYRO_DLPFCFG_5_7Hz   = 0x06,
    GYRO_DLPFCFG_361_4Hz = 0x07
} GYRO_DLPF_CFG_t;

typedef enum
{
    GYRO_FS_250DPS  = 0x00,
    GYRO_FS_500DPS  = 0x01,
    GYRO_FS_1000DPS = 0x02,
    GYRO_FS_2000DPS = 0x03
} GYRO_FS_t;

typedef enum
{
    GYRO_FCHOICE_BYPASS = 0x00,
    GYRO_FCHOICE_ENABLED = 0x01
} GYRO_FCHOICE_t;

//==============================================================================
// GYRO_CONFIG_2
//==============================================================================
#define GYRO_CONFIG_2(XGYRO_CTEN, YGYRO_CTEN, ZGYRO_CTEN, GYRO_AVGCFG) (\
    ((XGYRO_CTEN)   << 5) | \
    ((YGYRO_CTEN)   << 4) | \
    ((ZGYRO_CTEN)   << 3) | \
    ((GYRO_AVGCFG)  << 0)   \
)
typedef enum {
    GYRO_AVG_1X   = 0x00, // Default
    GYRO_AVG_2X   = 0x01,
    GYRO_AVG_4X   = 0x02,
    GYRO_AVG_8X   = 0x03,
    GYRO_AVG_16X  = 0x04,
    GYRO_AVG_32X  = 0x05,
    GYRO_AVG_64X  = 0x06,
    GYRO_AVG_128X = 0x07
} GYRO_AVG_CFG_t;

typedef enum {
    GYRO_SELFTEST_DISABLE = 0x00,
    GYRO_SELFTEST_ENABLE  = 0x01
} GYRO_SELFTEST_t;

//==============================================================================
// ACCEL_CONFIG
//==============================================================================
#define ACCEL_CONFIG(DLPF, FS, FCHOICE) (\
    ((DLPF)     << 3) | \
    ((FS)       << 1) | \
    ((FCHOICE)  << 0)   \
)
typedef enum {
    ACCEL_DLPFCFG_246_0Hz = 0x00, // หรือ 0x01 ก็ได้ Bandwidth เท่ากัน
    ACCEL_DLPFCFG_111_4Hz = 0x02,
    ACCEL_DLPFCFG_50_4Hz  = 0x03,
    ACCEL_DLPFCFG_23_9Hz  = 0x04,
    ACCEL_DLPFCFG_11_5Hz  = 0x05,
    ACCEL_DLPFCFG_5_7Hz   = 0x06,
    ACCEL_DLPFCFG_473_0Hz = 0x07
} ACCEL_DLPF_CFG_t;

typedef enum {
    ACCEL_FS_2g  = 0x00,
    ACCEL_FS_4g  = 0x01,
    ACCEL_FS_8g  = 0x02,
    ACCEL_FS_16g = 0x03
} ACCEL_FS_t;

typedef enum {
    ACCEL_FCHOICE_BYPASS    = 0x00,
    ACCEL_FCHOICE_ENABLED   = 0x01
} ACCEL_FCHOICE_t;

//==============================================================================
// ACCEL_CONFIG_2
//==============================================================================
#define ACCEL_CONFIG_2(AX_ST_EN_REG, AY_ST_EN_REG, AZ_ST_EN_REG, ACCEL_DEC3_CFG) (\
    (AX_ST_EN_REG   << 4) | \
    (AY_ST_EN_REG   << 3) | \
    (AZ_ST_EN_REG   << 2) | \
    (ACCEL_DEC3_CFG << 0)   \
)

typedef enum {
    ACCEL_DEC3_1X_OR_4X = 0x00, // เฉลี่ย 1 or 4 samples ขึ้นอยู่กับ ACCEL_FCHOICE (ดูตารางที่ 19)
    ACCEL_DEC3_8X       = 0x01, // เฉลี่ย 8 samples
    ACCEL_DEC3_16X      = 0x02, // เฉลี่ย 16 samples
    ACCEL_DEC3_32X      = 0x03  // เฉลี่ย 32 samples (ข้อมูลนิ่งขึ้นแต่ Latency สูง)
} ACCEL_DEC3_CFG_t;

typedef enum {
    ACCEL_SELFTEST_DISABLE = 0x00,
    ACCEL_SELFTEST_ENABLE  = 0x01
} ACCEL_SELFTEST_t;

// clang-format on
// --- Structs for Raw Data (int16_t) ---
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} raw_accel_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} raw_gyro_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} raw_mag_t;

typedef struct
{
    raw_accel_t accel;
    raw_gyro_t gyro;
    raw_mag_t mag;
    int16_t temp;
} raw_t;

// --- Axis Remapping Types ---
typedef enum
{
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2
} Axis_Sel_t;

typedef enum
{
    SIGN_POS = 1, // บวก (+)
    SIGN_NEG = -1 // ลบ (-)
} Axis_Sign_t;

typedef struct
{
    Axis_Sel_t src_axis; // เอาข้อมูลมาจากแกนไหนของ Sensor?
    Axis_Sign_t sign;    // ต้องกลับเครื่องหมายไหม?
} Axis_Map_t;

typedef struct
{
    Axis_Map_t x; // Body X (หน้ารถ) จะเอามาจากไหน?
    Axis_Map_t y; // Body Y (ขวารถ) จะเอามาจากไหน?
    Axis_Map_t z; // Body Z (พื้นรถ/เพดาน) จะเอามาจากไหน?
} Mounting_Matrix_t;

// --- Structs for Sensor Data ---
typedef struct
{
    float x;
    float y;
    float z;
} accel_t;

typedef struct
{
    float x;
    float y;
    float z;
} gyro_t;

typedef struct
{
    float x;
    float y;
    float z;
} mag_t;

typedef struct
{
    accel_t accel;
    gyro_t gyro;
    mag_t mag;
    float temp;
} sensor_t;

// --- Main Handle Struct ---
typedef struct
{
    raw_t raw;
    sensor_t sensor;

    // calibration data
    sensor_t bias;
    sensor_t scale;

    // mounting matrix
    Mounting_Matrix_t mounting;

    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *CS_GPIO_Port;
    uint16_t CS_Pin;

    uint8_t current_bank;
} ICM20948_HandleTypeDef;

// --- Public Function Access Struct ---
typedef struct
{
    void (*Init)(ICM20948_HandleTypeDef *icm);
    void (*SetMounting)(
        ICM20948_HandleTypeDef *icm,
        Axis_Sel_t x_src, Axis_Sign_t x_sign,
        Axis_Sel_t y_src, Axis_Sign_t y_sign,
        Axis_Sel_t z_src, Axis_Sign_t z_sign);
    void (*ReadData)(ICM20948_HandleTypeDef *icm);
    void (*CalibrateGyro)(ICM20948_HandleTypeDef *icm);
    void (*CalibrateMag)(ICM20948_HandleTypeDef *icm);
    void (*CalibrateAccel)(ICM20948_HandleTypeDef *icm);
    float (*CalculateHeading)(ICM20948_HandleTypeDef *icm);
} ICM20948_t;

extern ICM20948_t ICM20948;
#endif /* SRC_ICM20948_H_ */
