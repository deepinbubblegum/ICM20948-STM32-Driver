/*
 * ICM20948.c
 *
 * Created on: Nov 13, 2025
 * Author: chaiwit
 */

#include "ICM20948.h"

// Function prototypes
void ICM20948_Init(ICM20948_HandleTypeDef *icm);

// Static (private) functions
static void ICM20948_ReadData(ICM20948_HandleTypeDef *icm);
static HAL_StatusTypeDef ICM20948_SetBank(ICM20948_HandleTypeDef *icm, uint8_t bank);
static HAL_StatusTypeDef ICM20948_ReadRegister(ICM20948_HandleTypeDef *icm, uint8_t reg, uint8_t *pData);
static HAL_StatusTypeDef ICM20948_ReadRegisters(ICM20948_HandleTypeDef *icm, uint8_t reg, uint8_t *pData, uint8_t count);
static HAL_StatusTypeDef ICM20948_WriteRegister(ICM20948_HandleTypeDef *icm, uint8_t reg, uint8_t data);
static void AK09916_Init(ICM20948_HandleTypeDef *icm);
static uint8_t ICM20948_Mag_Read(ICM20948_HandleTypeDef *icm, uint8_t reg);
static void ICM20948_Mag_Write(ICM20948_HandleTypeDef *icm, uint8_t reg, uint8_t data);
static void ICM20948_CalibrateGyro(ICM20948_HandleTypeDef *icm);
static void ICM20948_CalibrateMag(ICM20948_HandleTypeDef *icm);
static void ICM20948_CalibrateAccel(ICM20948_HandleTypeDef *icm);
static void ICM20948_SetMounting(ICM20948_HandleTypeDef *icm,
                                 Axis_Sel_t x_src, Axis_Sign_t x_sign,
                                 Axis_Sel_t y_src, Axis_Sign_t y_sign,
                                 Axis_Sel_t z_src, Axis_Sign_t z_sign);
static float ICM20948_CalculateHeading(ICM20948_HandleTypeDef *icm);

// --- Public Function Struct ---
ICM20948_t ICM20948 = {
    .Init = ICM20948_Init,
    .SetMounting = ICM20948_SetMounting,
    .ReadData = ICM20948_ReadData,
    .CalibrateGyro = ICM20948_CalibrateGyro,
    .CalibrateMag = ICM20948_CalibrateMag,
    .CalibrateAccel = ICM20948_CalibrateAccel,
    .CalculateHeading = ICM20948_CalculateHeading,
};

//==============================================================================
// Public Functions
//==============================================================================
void ICM20948_Init(ICM20948_HandleTypeDef *icm)
{
    // ------------------------------------------------------------------
    // [USER CONFIG] Pre-calibrated values (Hardcoded)
    // WARNING: These values are specific to THIS sensor board only!
    // If you change the board, please run calibration functions again.
    // ------------------------------------------------------------------
    // configure calibration data
    // Reset Bias (0) default values 0
    icm->bias.gyro.x = -8.2528f;
    icm->bias.gyro.y = 3.1247f;
    icm->bias.gyro.z = 5.9230f;

    icm->bias.mag.x = -69.50f;
    icm->bias.mag.y = 125.50f;
    icm->bias.mag.z = 212.50f;

    icm->bias.accel.x = -18.9f;
    icm->bias.accel.y = -14.8f;
    icm->bias.accel.z = 141.7f;

    // Set Scale (1) default values 1
    icm->scale.gyro.x = 1.0f;
    icm->scale.gyro.y = 1.0f;
    icm->scale.gyro.z = 1.0f;

    icm->scale.mag.x = 0.8822f;
    icm->scale.mag.y = 1.0438f;
    icm->scale.mag.z = 1.1009f;

    icm->scale.accel.x = 1.0021f;
    icm->scale.accel.y = 1.0010f;
    icm->scale.accel.z = 0.9980f;

    // default mounting matrix (no rotation)
    ICM20948_SetMounting(icm, AXIS_X, SIGN_POS, AXIS_Y, SIGN_POS, AXIS_Z, SIGN_POS);

    uint8_t who_am_i;
    icm->current_bank = 0xFF; // Invalid bank to force setting bank at first

    // Switch to Bank 0
    ICM20948_SetBank(icm, 0);
    ICM20948_ReadRegister(icm, REG_B0_WHO_AM_I, &who_am_i);
    if (who_am_i != 0xEA)
        printf("ICM20948 WHO_AM_I MISMATCH: 0x%02X\r\n", who_am_i);
    else
        printf("ICM20948 WHO_AM_I: 0x%02X\r\n", who_am_i);

    // --- 1. Software Reset ---
    // เพื่อ Reset และตั้งค่า Sleep (8.4 PWR_MGMT_1)
    ICM20948_WriteRegister(icm, REG_B0_PWR_MGMT_1, 0x80);
    HAL_Delay(150); // รอประมาณ 100 ms เพื่อให้ชิป Reset เสร็จสมบูรณ์

    // --- 2. Wake Up and Auto Select Clock ---
    ICM20948_WriteRegister(icm, REG_B0_PWR_MGMT_1, 0x01);
    HAL_Delay(50);

    // --- 3. Disable I2C (ใช้ SPI อย่างเดียว) ---
    ICM20948_WriteRegister(icm, REG_B0_USER_CTRL, 0x30); // I2C_IF_DIS bit
    HAL_Delay(50);
    // สั่ง Reset I2C Master (Bit 1) ทับลงไป (0x30 | 0x02)
    ICM20948_WriteRegister(icm, REG_B0_USER_CTRL, 0x32);
    HAL_Delay(50);

    // Switch to Bank 2
    ICM20948_SetBank(icm, 2);
    // --- 4. ตั้งค่า Accel/Gyro (BANK 2) ---
    // เปิด ODR Alignment
    ICM20948_WriteRegister(icm, REG_B2_ODR_ALIGN_EN, 0x01);

    // (ใช้ ODR ~50Hz)
    ICM20948_WriteRegister(icm, REG_B2_GYRO_SMPLRT_DIV, 21); // 1125/(1+21) ~ 51 Hz
    // ±2000dps, DLPFCFG=51.2Hz, FCHOICE=enabled
    ICM20948_WriteRegister(icm, REG_B2_GYRO_CONFIG_1, GYRO_CONFIG_1(GYRO_DLPFCFG_51_2Hz, GYRO_FS_2000DPS, GYRO_FCHOICE_ENABLED));
    // X,Y,Z gyro self-test disabled, 1x avg
    ICM20948_WriteRegister(icm, REG_B2_GYRO_CONFIG_2, GYRO_CONFIG_2(GYRO_SELFTEST_DISABLE, GYRO_SELFTEST_DISABLE, GYRO_SELFTEST_DISABLE, GYRO_AVG_1X));

    // MSB for ACCEL sample rate div
    ICM20948_WriteRegister(icm, REG_B2_ACCEL_SMPLRT_DIV_1, 0x00);
    // LSB for ACCEL sample rate div
    ICM20948_WriteRegister(icm, REG_B2_ACCEL_SMPLRT_DIV_2, 21); // 1125/(1+21) ~ 51 Hz
    // ±4g, DLPF=50.4Hz
    ICM20948_WriteRegister(icm, REG_B2_ACCEL_CONFIG, ACCEL_CONFIG(ACCEL_DLPFCFG_50_4Hz, ACCEL_FS_4g, ACCEL_FCHOICE_ENABLED));
    // X,Y,Z accel self-test disabled, avg 1 or 4 samples
    ICM20948_WriteRegister(icm, REG_B2_ACCEL_CONFIG_2, ACCEL_CONFIG_2(ACCEL_SELFTEST_DISABLE, ACCEL_SELFTEST_DISABLE, ACCEL_SELFTEST_DISABLE, ACCEL_DEC3_1X_OR_4X));

    // Initialize AK09916 (Magnetometer)
    AK09916_Init(icm);

    ICM20948_SetBank(icm, 0); // Switch back to Bank 0
}

void AK09916_Init(ICM20948_HandleTypeDef *icm)
{
    // 1. Config I2C Master Clock (Bank 3)
    ICM20948_SetBank(icm, 3);
    ICM20948_WriteRegister(icm, REG_B3_I2C_MST_CTRL, 0x07);       // [CHANGE] ลดความเร็ว I2C ลงเหลือ ~400kHz (0x07) เพื่อความชัวร์
    ICM20948_WriteRegister(icm, REG_B3_I2C_MST_ODR_CONFIG, 0x00); // Sync with Gyro

    // 2. Force Mag Reset
    ICM20948_Mag_Write(icm, REG_MAG_CNTL2, 0x00); // Power Down
    HAL_Delay(10);
    ICM20948_Mag_Write(icm, REG_MAG_CNTL3, 0x01); // Soft Reset
    HAL_Delay(100);                               // รอให้ Mag Reset เสร็จจริง

    // 3. Check Mag ID
    uint8_t mag_id = 0;
    for (int i = 0; i < 5; i++)
    {
        mag_id = ICM20948_Mag_Read(icm, REG_MAG_WIA2);
        if (mag_id == 0x09)
            break;
        HAL_Delay(10);
    }

    if (mag_id != 0x09)
        printf("Mag ID Error: %02X\r\n", mag_id);
    else
        printf("Mag ID OK: 0x09\r\n");

    // 4. Set Mag Mode (Continuous 100Hz)
    ICM20948_Mag_Write(icm, REG_MAG_CNTL2, 0x08);
    HAL_Delay(20); // [IMPORTANT] รอให้ Mag ตื่นและเริ่มเก็บ Sample แรก

    // ============================================================
    // [CRITICAL FIX] Reset I2C Master logic & Disable Slave 4
    // ============================================================
    ICM20948_SetBank(icm, 3);

    // Disable Slave 4 Explicitly (ปิด Slave 4 เพื่อไม่ให้กวน Slave 0)
    ICM20948_WriteRegister(icm, REG_B3_I2C_SLV4_CTRL, 0x00);
    HAL_Delay(5);

    // Reset I2C Master State Machine (โดยไม่ Reset Register อื่น)
    // กลับไป Bank 0 เพื่อเข้าถึง USER_CTRL
    ICM20948_SetBank(icm, 0);
    // BIT 5 (I2C_MST_EN) = 1, BIT 4 (I2C_IF_DIS) = 1, BIT 1 (I2C_MST_RST) = 1
    ICM20948_WriteRegister(icm, REG_B0_USER_CTRL, 0x32);
    HAL_Delay(20); // รอให้ Reset เสร็จ

    // ============================================================
    // 5. Setup Auto-Read (Slave 0)
    // ============================================================
    ICM20948_SetBank(icm, 3);

    // Address: Read (0x80) from Mag (0x0C)
    ICM20948_WriteRegister(icm, REG_B3_I2C_SLV0_ADDR, REG_MAG_SLAVE_ADDR | 0x80);

    // Start Register: ST1 (0x10)
    ICM20948_WriteRegister(icm, REG_B3_I2C_SLV0_REG, REG_MAG_ST1);

    // Enable Slave 0, Length 9 bytes (ST1 + Data + ST2)
    // Swap Disabled (เพราะ Code อ่าน Little Endian เองแล้ว)
    ICM20948_WriteRegister(icm, REG_B3_I2C_SLV0_CTRL, 0x89);

    ICM20948_SetBank(icm, 0);
}

static void ICM20948_ReadData(ICM20948_HandleTypeDef *icm)
{
    uint8_t buffer[24];
    ICM20948_ReadRegisters(icm, REG_B0_ACCEL_XOUT_H, buffer, 24);

    // Accel and Gyro Data big endian
    icm->raw.accel.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    icm->raw.accel.y = (int16_t)((buffer[2] << 8) | buffer[3]);
    icm->raw.accel.z = (int16_t)((buffer[4] << 8) | buffer[5]);

    icm->raw.gyro.x = (int16_t)((buffer[6] << 8) | buffer[7]);
    icm->raw.gyro.y = (int16_t)((buffer[8] << 8) | buffer[9]);
    icm->raw.gyro.z = (int16_t)((buffer[10] << 8) | buffer[11]);

    // Temperature (ถ้าไม่ใช้ก็ข้ามไป) 12 - 13
    icm->raw.temp = (int16_t)((buffer[12] << 8) | buffer[13]);

    // Magnetometer Data little endian
    icm->raw.mag.x = (int16_t)((buffer[16] << 8) | buffer[15]);
    icm->raw.mag.y = (int16_t)((buffer[18] << 8) | buffer[17]);
    icm->raw.mag.z = (int16_t)((buffer[20] << 8) | buffer[19]);
    // printf("ST2 = %02X\n", buffer[22]);

    // Sensor (x_s, y_s, z_s)
    float ax_s, ay_s, az_s;
    float gx_s, gy_s, gz_s;
    float mx_s, my_s, mz_s;

    // --- Accel (Sensor Frame) ---
    ax_s = ((float)icm->raw.accel.x - icm->bias.accel.x) * icm->scale.accel.x / ACCEL_SENSITIVITY;
    ay_s = ((float)icm->raw.accel.y - icm->bias.accel.y) * icm->scale.accel.y / ACCEL_SENSITIVITY;
    az_s = ((float)icm->raw.accel.z - icm->bias.accel.z) * icm->scale.accel.z / ACCEL_SENSITIVITY;

    // --- Gyro (Sensor Frame) ---
    gx_s = ((float)icm->raw.gyro.x - icm->bias.gyro.x) / GYRO_SENSITIVITY;
    gy_s = ((float)icm->raw.gyro.y - icm->bias.gyro.y) / GYRO_SENSITIVITY;
    gz_s = ((float)icm->raw.gyro.z - icm->bias.gyro.z) / GYRO_SENSITIVITY;

    // // --- Mag (Sensor Frame) ---
    mx_s = ((float)icm->raw.mag.y - icm->bias.mag.y) * icm->scale.mag.y * MAG_SENSITIVITY;
    my_s = ((float)icm->raw.mag.x - icm->bias.mag.x) * icm->scale.mag.x * MAG_SENSITIVITY;
    mz_s = ((float)icm->raw.mag.z - icm->bias.mag.z) * icm->scale.mag.z * MAG_SENSITIVITY * -1.0f;

    // เก็บใส่ Array ชั่วคราว
    float a_tmp[3] = {ax_s, ay_s, az_s};
    float g_tmp[3] = {gx_s, gy_s, gz_s};
    float m_tmp[3] = {mx_s, my_s, mz_s};

    // Map Accel to Body Frame
    icm->sensor.accel.x = a_tmp[icm->mounting.x.src_axis] * (float)icm->mounting.x.sign;
    icm->sensor.accel.y = a_tmp[icm->mounting.y.src_axis] * (float)icm->mounting.y.sign;
    icm->sensor.accel.z = a_tmp[icm->mounting.z.src_axis] * (float)icm->mounting.z.sign;

    // Map Gyro to Body Frame
    icm->sensor.gyro.x = g_tmp[icm->mounting.x.src_axis] * (float)icm->mounting.x.sign;
    icm->sensor.gyro.y = g_tmp[icm->mounting.y.src_axis] * (float)icm->mounting.y.sign;
    icm->sensor.gyro.z = g_tmp[icm->mounting.z.src_axis] * (float)icm->mounting.z.sign;

    // Map Mag to Body Frame
    icm->sensor.mag.x = m_tmp[icm->mounting.x.src_axis] * (float)icm->mounting.x.sign;
    icm->sensor.mag.y = m_tmp[icm->mounting.y.src_axis] * (float)icm->mounting.y.sign;
    icm->sensor.mag.z = m_tmp[icm->mounting.z.src_axis] * (float)icm->mounting.z.sign;

    // Temperature
    icm->sensor.temp = (float)icm->raw.temp / 333.87f + 21.0f;
}

static void ICM20948_Mag_Write(ICM20948_HandleTypeDef *icm, uint8_t reg, uint8_t data)
{
    ICM20948_SetBank(icm, 3);
    ICM20948_WriteRegister(icm, REG_B3_I2C_SLV4_ADDR, REG_MAG_SLAVE_ADDR); // Write
    ICM20948_WriteRegister(icm, REG_B3_I2C_SLV4_REG, reg);
    ICM20948_WriteRegister(icm, REG_B3_I2C_SLV4_DO, data);
    ICM20948_WriteRegister(icm, REG_B3_I2C_SLV4_CTRL, 0x80); // Enable
    HAL_Delay(2);
}

static uint8_t ICM20948_Mag_Read(ICM20948_HandleTypeDef *icm, uint8_t reg)
{
    ICM20948_SetBank(icm, 3);
    ICM20948_WriteRegister(icm, REG_B3_I2C_SLV4_ADDR, REG_MAG_SLAVE_ADDR | 0x80); // Read
    ICM20948_WriteRegister(icm, REG_B3_I2C_SLV4_REG, reg);
    ICM20948_WriteRegister(icm, REG_B3_I2C_SLV4_CTRL, 0x80); // Enable
    HAL_Delay(2);
    uint8_t data;
    ICM20948_ReadRegister(icm, REG_B3_I2C_SLV4_DI, &data);
    return data;
}

static HAL_StatusTypeDef ICM20948_SetBank(ICM20948_HandleTypeDef *icm, uint8_t bank)
{
    if (icm->current_bank != bank)
    {
        HAL_StatusTypeDef status;
        status = ICM20948_WriteRegister(icm, REG_BANK_SEL, (bank << 4));
        if (status == HAL_OK)
        {
            icm->current_bank = bank;
            for (volatile int i = 0; i < 100; i++)
                ;
            return HAL_OK;
        }
        else
        {
            return status;
        }
    }
    return HAL_OK;
}

static HAL_StatusTypeDef ICM20948_ReadRegister(ICM20948_HandleTypeDef *icm, uint8_t reg, uint8_t *pData)
{
    HAL_StatusTypeDef status;

    // เตรียม Buffer 2 ช่อง
    // txData[0] = Address | Read Bit (0x80)
    // txData[1] = Dummy Byte (0x00) เพื่อดัน Clock ให้ Data ไหลกลับมา
    uint8_t txData[2] = {reg | 0x80, 0x00};
    uint8_t rxData[2] = {0, 0};

    // 1. CS Low
    HAL_GPIO_WritePin(icm->CS_GPIO_Port, icm->CS_Pin, GPIO_PIN_RESET);

    // 2. ส่งและรับพร้อมกัน 2 Bytes
    // Byte แรก: เราส่ง Address -> รับขยะ (หรือ Status)
    // Byte สอง: เราส่ง Dummy   -> รับ Data จริง
    status = HAL_SPI_TransmitReceive(icm->hspi, txData, rxData, 2, HAL_MAX_DELAY);

    // 3. CS High
    HAL_GPIO_WritePin(icm->CS_GPIO_Port, icm->CS_Pin, GPIO_PIN_SET);

    // 4. เอาค่า Byte ที่ 2 มาใช้
    if (status == HAL_OK)
    {
        *pData = rxData[1];
    }

    return status;
}

static HAL_StatusTypeDef ICM20948_ReadRegisters(ICM20948_HandleTypeDef *icm, uint8_t reg, uint8_t *pData, uint8_t count)
{
    HAL_StatusTypeDef status;
    uint8_t txData = reg | 0x80; // Address + Read bit

    // 1. ดึง CS ลงเพื่อเริ่มคุย
    HAL_GPIO_WritePin(icm->CS_GPIO_Port, icm->CS_Pin, GPIO_PIN_RESET);

    // 2. ส่ง Address 1 Byte ไปก่อน
    status = HAL_SPI_Transmit(icm->hspi, &txData, 1, HAL_MAX_DELAY);

    // 3. ถ้าส่ง Address ผ่าน, ให้รับข้อมูลต่อทันที (โดยยังไม่ปล่อย CS)
    // HAL_SPI_Receive จะส่ง Dummy clock ให้เองอัตโนมัติ
    if (status == HAL_OK)
    {
        status = HAL_SPI_Receive(icm->hspi, pData, count, HAL_MAX_DELAY);
    }

    // 4. ปล่อย CS ขึ้นเมื่อเสร็จทุกอย่าง
    HAL_GPIO_WritePin(icm->CS_GPIO_Port, icm->CS_Pin, GPIO_PIN_SET);

    return status;
}

static HAL_StatusTypeDef ICM20948_WriteRegister(ICM20948_HandleTypeDef *icm, uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t txData[2];
    txData[0] = reg | WRITE;
    txData[1] = data;
    HAL_GPIO_WritePin(icm->CS_GPIO_Port, icm->CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(icm->hspi, txData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(icm->CS_GPIO_Port, icm->CS_Pin, GPIO_PIN_SET);
    for (volatile int i = 0; i < 100; i++)
        ;
    return status;
}

// =============================================================================
// Calibration Functions
// =============================================================================
/**
 * @brief  Calibrate Gyroscope (Sensor must be STATIONARY)
 * หาค่า Error ตอนอยู่นิ่งๆ เพื่อนำไปลบออก (Zero-rate bias)
 */
static void ICM20948_CalibrateGyro(ICM20948_HandleTypeDef *icm)
{
    printf("Starting Gyro Calibration... KEEP STATIONARY!\r\n");

    int32_t gyro_sum[3] = {0, 0, 0};
    const int samples = 10000;

    for (int i = 0; i < samples; i++)
    {
        ICM20948_ReadData(icm);
        gyro_sum[0] += icm->raw.gyro.x;
        gyro_sum[1] += icm->raw.gyro.y;
        gyro_sum[2] += icm->raw.gyro.z;
        HAL_Delay(5);
    }

    icm->bias.gyro.x = (float)gyro_sum[0] / samples;
    icm->bias.gyro.y = (float)gyro_sum[1] / samples;
    icm->bias.gyro.z = (float)gyro_sum[2] / samples;
    printf("Gyro Bias: X=%.4f, Y=%.4f, Z=%.4f\r\n", icm->bias.gyro.x, icm->bias.gyro.y, icm->bias.gyro.z);
}

/**
 * @brief  Calibrate Magnetometer (Must ROTATE sensor in Figure-8)
 * หาค่า Min/Max เพื่อคำนวณ Hard Iron Offset
 */
static void ICM20948_CalibrateMag(ICM20948_HandleTypeDef *icm)
{
    printf("Starting Mag Calibration... ROTATE SENSOR in Figure-8 for 30 seconds!\r\n");

    int16_t mag_min[3] = {32000, 32000, 32000};
    int16_t mag_max[3] = {-32000, -32000, -32000};

    uint32_t start_tick = HAL_GetTick();

    // วนลูป 15 วินาที
    while ((HAL_GetTick() - start_tick) < 30000)
    {
        ICM20948_ReadData(icm);

        // Check X
        if (icm->raw.mag.x < mag_min[0])
            mag_min[0] = icm->raw.mag.x;
        if (icm->raw.mag.x > mag_max[0])
            mag_max[0] = icm->raw.mag.x;

        // Check Y
        if (icm->raw.mag.y < mag_min[1])
            mag_min[1] = icm->raw.mag.y;
        if (icm->raw.mag.y > mag_max[1])
            mag_max[1] = icm->raw.mag.y;

        // Check Z
        if (icm->raw.mag.z < mag_min[2])
            mag_min[2] = icm->raw.mag.z;
        if (icm->raw.mag.z > mag_max[2])
            mag_max[2] = icm->raw.mag.z;

        HAL_Delay(20); // Sampling rate ~50Hz
    }

    // คำนวณ Hard Iron Offset (Bias) -> (Max + Min) / 2
    icm->bias.mag.x = (float)(mag_max[0] + mag_min[0]) / 2.0f;
    icm->bias.mag.y = (float)(mag_max[1] + mag_min[1]) / 2.0f;
    icm->bias.mag.z = (float)(mag_max[2] + mag_min[2]) / 2.0f;

    // คำนวณ Soft Iron Scale (Simple version) -> (Max - Min) / 2
    // หมายเหตุ: วิธีนี้เป็นวิธีพื้นฐาน (Simple scaling) เพื่อให้แกนเท่ากัน
    float chord_x = ((float)(mag_max[0] - mag_min[0])) / 2.0f;
    float chord_y = ((float)(mag_max[1] - mag_min[1])) / 2.0f;
    float chord_z = ((float)(mag_max[2] - mag_min[2])) / 2.0f;

    float avg_chord = (chord_x + chord_y + chord_z) / 3.0f;

    if (chord_x == 0.0f) icm->scale.mag.x = 1.0f;
    else icm->scale.mag.x = avg_chord / chord_x;

    if (chord_y == 0.0f) icm->scale.mag.y = 1.0f;
    else icm->scale.mag.y = avg_chord / chord_y;

    if (chord_z == 0.0f) icm->scale.mag.z = 1.0f;
    else icm->scale.mag.z = avg_chord / chord_z;

    printf("Mag Bias: X=%.4f, Y=%.4f, Z=%.4f\r\n", icm->bias.mag.x, icm->bias.mag.y, icm->bias.mag.z);
    printf("Mag Scale: X=%.4f, Y=%.4f, Z=%.4f\r\n", icm->scale.mag.x, icm->scale.mag.y, icm->scale.mag.z);
}

/**
 * @brief  Calibrate Accelerometer (6-Point Method)
 * ผู้ใช้ต้องวางเซนเซอร์ 6 ท่าตามคำสั่งใน Terminal
 */
static void ICM20948_CalibrateAccel(ICM20948_HandleTypeDef *icm)
{
    float acc_sum[3];
    float acc_plus[3], acc_minus[3]; // เก็บค่าตอน +1g และ -1g ของแต่ละแกน
    const int samples = 3000;

    printf("\r\n=== START ACCEL CALIBRATION (6-POINT) ===\r\n");
    printf("I will ask you to place sensor in 6 positions.\r\n");
    printf("Hold it STEADY for 2 seconds each time.\r\n");
    HAL_Delay(2000);

    // ---------------------------------------------------------
    // 1. Z Axis Calibration
    // ---------------------------------------------------------

    // Position 1: Z+ (วางราบ หงายท้อง)
    printf("\r\n[1/6] Place FLAT (Z pointing UP) -> Press 'Enter' or Wait 10s...\r\n");
    HAL_Delay(10000);
    printf("Sampling Z+...\r\n");

    acc_sum[0] = 0;
    acc_sum[1] = 0;
    acc_sum[2] = 0;
    for (int i = 0; i < samples; i++)
    {
        ICM20948_ReadData(icm);
        // แปลงเป็น g ดิบๆ (ยังไม่เข้า ProcessData) เพื่อหาค่า Bias ดิบ
        acc_sum[2] += (float)icm->raw.accel.z / 8192.0f; // Scale 4g
        HAL_Delay(5);
    }
    acc_plus[2] = acc_sum[2] / samples;
    printf("Done Z+ = %.3f g\r\n", acc_plus[2]);

    // Position 2: Z- (คว่ำหน้า)
    printf("\r\n[2/6] Place UPSIDE DOWN (Z pointing DOWN) -> Wait 10s...\r\n");
    HAL_Delay(10000);
    printf("Sampling Z-...\r\n");

    acc_sum[2] = 0;
    for (int i = 0; i < samples; i++)
    {
        ICM20948_ReadData(icm);
        acc_sum[2] += (float)icm->raw.accel.z / 8192.0f;
        HAL_Delay(5);
    }
    acc_minus[2] = acc_sum[2] / samples;
    printf("Done Z- = %.3f g\r\n", acc_minus[2]);

    // ---------------------------------------------------------
    // 2. X Axis Calibration
    // ---------------------------------------------------------

    // Position 3: X+ (เอาหัวลูกศร X ตั้งขึ้นฟ้า)
    printf("\r\n[3/6] Point X Axis UP (Vertical) -> Wait 10s...\r\n");
    HAL_Delay(10000);
    printf("Sampling X+...\r\n");

    acc_sum[0] = 0;
    for (int i = 0; i < samples; i++)
    {
        ICM20948_ReadData(icm);
        acc_sum[0] += (float)icm->raw.accel.x / 8192.0f;
        HAL_Delay(5);
    }
    acc_plus[0] = acc_sum[0] / samples;
    printf("Done X+ = %.3f g\r\n", acc_plus[0]);

    // Position 4: X- (เอาหัวลูกศร X ทิ่มลงดิน)
    printf("\r\n[4/6] Point X Axis DOWN (Vertical) -> Wait 10s...\r\n");
    HAL_Delay(10000);
    printf("Sampling X-...\r\n");

    acc_sum[0] = 0;
    for (int i = 0; i < samples; i++)
    {
        ICM20948_ReadData(icm);
        acc_sum[0] += (float)icm->raw.accel.x / 8192.0f;
        HAL_Delay(5);
    }
    acc_minus[0] = acc_sum[0] / samples;
    printf("Done X- = %.3f g\r\n", acc_minus[0]);

    // ---------------------------------------------------------
    // 3. Y Axis Calibration
    // ---------------------------------------------------------
    // Position 5: Y+ (ตะแคงให้ Y ชี้ขึ้นฟ้า)
    printf("\r\n[5/6] Point Y Axis UP (Vertical) -> Wait 10s...\r\n");
    HAL_Delay(10000);
    printf("Sampling Y+...\r\n");

    acc_sum[1] = 0;
    for (int i = 0; i < samples; i++)
    {
        ICM20948_ReadData(icm);
        acc_sum[1] += (float)icm->raw.accel.y / 8192.0f;
        HAL_Delay(5);
    }
    acc_plus[1] = acc_sum[1] / samples;
    printf("Done Y+ = %.3f g\r\n", acc_plus[1]);

    // Position 6: Y- (ตะแคงให้ Y ชี้ลงดิน)
    printf("\r\n[6/6] Point Y Axis DOWN (Vertical) -> Wait 10s...\r\n");
    HAL_Delay(10000);
    printf("Sampling Y-...\r\n");

    acc_sum[1] = 0;
    for (int i = 0; i < samples; i++)
    {
        ICM20948_ReadData(icm);
        acc_sum[1] += (float)icm->raw.accel.y / 8192.0f;
        HAL_Delay(5);
    }
    acc_minus[1] = acc_sum[1] / samples;
    printf("Done Y- = %.3f g\r\n", acc_minus[1]);

    // ---------------------------------------------------------
    // Calculate Bias & Scale
    // ---------------------------------------------------------
    // สูตร:
    // Bias  = (Plus + Minus) / 2
    // Scale = (Plus - Minus) / 2  (ควรจะได้ใกล้เคียง 1g)
    // Scale Factor = 1.0 / Measured_Scale

    // X Axis
    float x_bias_g = (acc_plus[0] + acc_minus[0]) / 2.0f;
    float x_scale_g = (acc_plus[0] - acc_minus[0]) / 2.0f;

    // Y Axis
    float y_bias_g = (acc_plus[1] + acc_minus[1]) / 2.0f;
    float y_scale_g = (acc_plus[1] - acc_minus[1]) / 2.0f;

    // Z Axis
    float z_bias_g = (acc_plus[2] + acc_minus[2]) / 2.0f;
    float z_scale_g = (acc_plus[2] - acc_minus[2]) / 2.0f;

    icm->bias.accel.x = x_bias_g * 8192.0f; // เก็บเป็นหน่วย Raw LSB เพื่อให้เข้าสูตรเดิมง่าย
    icm->bias.accel.y = y_bias_g * 8192.0f;
    icm->bias.accel.z = z_bias_g * 8192.0f;

    if (x_scale_g == 0.0f) icm->scale.accel.x = 1.0f;
    else icm->scale.accel.x = 1.0f / x_scale_g;

    if (y_scale_g == 0.0f) icm->scale.accel.y = 1.0f;
    else icm->scale.accel.y = 1.0f / y_scale_g;

    if (z_scale_g == 0.0f) icm->scale.accel.z = 1.0f;
    else icm->scale.accel.z = 1.0f / z_scale_g;

    printf("\r\n=== ACCEL RESULT ===\r\n");
    printf("Bias (LSB): X=%.1f, Y=%.1f, Z=%.1f\r\n", icm->bias.accel.x, icm->bias.accel.y, icm->bias.accel.z);
    printf("Scale Factor: X=%.4f, Y=%.4f, Z=%.4f\r\n", icm->scale.accel.x, icm->scale.accel.y, icm->scale.accel.z);
}

static void ICM20948_SetMounting(ICM20948_HandleTypeDef *icm, Axis_Sel_t x_src, Axis_Sign_t x_sign, Axis_Sel_t y_src, Axis_Sign_t y_sign, Axis_Sel_t z_src, Axis_Sign_t z_sign)
{
    // แกน X
    icm->mounting.x.src_axis = x_src;
    icm->mounting.x.sign = x_sign;

    // แกน Y
    icm->mounting.y.src_axis = y_src;
    icm->mounting.y.sign = y_sign;

    // แกน Z
    icm->mounting.z.src_axis = z_src;
    icm->mounting.z.sign = z_sign;
}

// ฟังก์ชันคำนวณทิศที่ถูกต้อง (รองรับการเอียง + หมุนตามเข็มนาฬิกา)
static float ICM20948_CalculateHeading(ICM20948_HandleTypeDef *icm) {
    // 1. ดึงค่า Body Frame (ที่ Remap มาแล้ว)
    float ax = icm->sensor.accel.x;
    float ay = icm->sensor.accel.y;
    float az = icm->sensor.accel.z;
    float mx = icm->sensor.mag.x;
    float my = icm->sensor.mag.y;
    float mz = icm->sensor.mag.z;

    // 2. คำนวณมุมเอียงของรถ (Pitch & Roll) จากแรงโน้มถ่วง
    // สูตรนี้ใช้ได้เสมอเพราะเรา Remap แกน Accel มาให้ตรงกับ Body แล้ว
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    // 3. หมุนสนามแม่เหล็กกลับมาให้ขนานกับพื้นโลก (Tilt Compensation)
    float cos_roll  = cosf(roll);
    float sin_roll  = sinf(roll);
    float cos_pitch = cosf(pitch);
    float sin_pitch = sinf(pitch);

    // คำนวณองค์ประกอบแม่เหล็กในแนวราบ (Horizontal Component)
    float Xh = mx * cos_pitch + mz * sin_pitch;
    float Yh = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;

    // 4. คำนวณทิศ (Heading)
    // [สำคัญ] ต้องใช้ -Yh เพื่อให้มุมหมุน "ตามเข็มนาฬิกา" (Clockwise)
    // 0=เหนือ, 90=ตะวันออก, 180=ใต้, 270=ตะวันตก
    float heading = atan2f(-Yh, Xh) * (180.0f / 3.14159265f);

    // 5. ปรับช่วงค่าลบให้เป็น 0-360
    if (heading < 0) {
        heading += 360.0f;
    }

    return heading;
}