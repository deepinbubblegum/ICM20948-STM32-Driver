# ICM-20948 STM32 Driver: From Noisy Data to Robot-Ready Orientation

[![Interface](https://img.shields.io/badge/Interface-SPI-blue?style=for-the-badge)](https://www.st.com/en/development-tools/stm32cubeide.html)

An advanced STM32 driver for the TDK InvenSense ICM-20948 9-DOF sensor. This driver provides a complete software suite to transform the sensor's raw, chaotic output into a stable, accurate, and intuitive data stream ready for any robotics, drone, or motion-tracking project.

This project was developed with STM32CubeIDE and is configured for an STM32F746ZGTX.

---

## Key Features

This driver is more than just a register reader. It's a full-fledged processing suite.

### 1. Complete 9-DOF Data Access
The driver unifies all 9 axes (plus temperature) from the three internal sensors (Accelerometer, Gyroscope, Magnetometer) into a single, easy-to-use data structure.

### 2. Advanced Calibration Suite
A guided, three-stage process corrects all common sensor errors, turning raw values into physically accurate measurements.
-   **Gyroscope:** Calibrates the sensor's bias by measuring noise while stationary.
-   **Accelerometer:** A 6-point tumbling calibration to find bias and scale factors for each axis.
-   **Magnetometer:** A figure-8 motion routine to calculate Hard Iron (bias) and Soft Iron (scale/distortion) corrections.

### 3. Axis Remapping Engine
Easily align the sensor's physical orientation (Sensor Frame) with your vehicle's coordinate system (Body Frame). A simple `SetMounting()` function allows you to define which sensor axis corresponds to your robot's "Front," "Right," and "Up" directions, including inversions.

---

## Hardware & Software

*   **Microcontroller:** STM32F746ZGTX (easily adaptable to other STM32 models)
*   **Sensor:** TDK InvenSense ICM-20948
*   **Interface:** SPI (currently supported)
*   **IDE:** [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)

---

## Getting Started

1.  Clone this repository.
2.  Open the project in STM32CubeIDE.
3.  Connect your hardware (STM32 board and ICM-20948 sensor via SPI).
4.  Build the project (`Project` -> `Build All`).
5.  Run the debugger (`Run` -> `Debug`) to flash the microcontroller.

---

## Usage & Code Examples

### 1. Standard Data Reading

Include the header and define the handle with your hardware configuration. The main loop consists of calling `ReadData` and then using the processed values.

```c
#include "ICM20948.h"

// Define Handle with your Hardware Config
ICM20948_HandleTypeDef ICM = {
    .hspi = &hspi5,
    .CS_GPIO_Port = GPIOA,
    .CS_Pin = GPIO_PIN_4
};

void main() {
    // 1. Initialize Sensor & Load Calibration
    ICM20948.Init(&ICM);

    // 2. (Optional) Configure Sensor Mounting Orientation
    ICM20948.SetMounting(&ICM,
        AXIS_Y, SIGN_POS,  // Robot Front (X) = Sensor Y
        AXIS_X, SIGN_NEG,  // Robot Right (Y) = Sensor -X
        AXIS_Z, SIGN_POS   // Robot Up (Z)    = Sensor Z
    );

    while (1) {
        // 3. Read & Process Data
        ICM20948.ReadData(&ICM);
        
        // Use the data (already calibrated and re-oriented)
        float accel_x = ICM.sensor.accel.x; // in g
        float gyro_z = ICM.sensor.gyro.z;   // in dps
        
        printf("Accel X: %.2f g\r\n", accel_x);
        HAL_Delay(5);
    }
}
```

### 2. Performing Calibration

Run these functions once and save the output. The driver will guide you through the process via a serial terminal.

```c
void RunCalibration() {
    printf("Starting Full Sensor Calibration...\r\n");

    // 1. Gyro: Place sensor FLAT and STATIONARY
    ICM20948.CalibrateGyro(&ICM);

    // 2. Accel: Follow terminal prompts (6 positions required)
    ICM20948.CalibrateAccel(&ICM);

    // 3. Mag: Wave sensor in a Figure-8 motion until complete
    ICM20948.CalibrateMag(&ICM);
    
    printf("Calibration Complete! Copy the values below.\r\n");
    // The results will be printed to your terminal.
}
```

### 3. Saving Calibration Data

After running the calibration, copy the output values from your terminal and paste them into the `ICM20948_Init` function in `ICM20948.c`. This hardcodes the calibration, so you don't need to run the routine on every power-up.

```c
void ICM20948_Init(ICM20948_HandleTypeDef *icm)
{
    // ------------------------------------------------------------------
    // [USER CONFIG] Paste pre-calibrated values here
    // ------------------------------------------------------------------
    
    // Gyro Bias
    icm->bias.gyro.x = -8.2528f;
    icm->bias.gyro.y = 3.1247f;
    icm->bias.gyro.z = 5.9230f;

    // Mag Bias (Hard Iron)
    icm->bias.mag.x = -215.5f;
    // ... paste all other bias and scale values ...

    // ... rest of init code ...
}
```
## Documentation

For a more detailed explanation of the driver, its features, and the implementation, please visit the project page:
[ICM-20948 STM32 Driver Documentation](https://deepinbubblegum.github.io/ICM20948-STM32-Driver/)

