# STM32G474RE Multi-VL53L1X

Multi-sensor VL53L1X Time-of-Flight ranging implementation for the STM32G474RE.

This project supports up to **5 VL53L1X sensors** using multiple I2C buses and independent XSHUT control. Each sensor is initialized sequentially, assigned a unique I2C address, and then read using a round-robin polling method.

<img width="897" height="223" alt="STM32G474RE VL53L1X" src="https://github.com/user-attachments/assets/f5ec8513-2dc4-4dea-ab5a-206b70ebfaf7" />

## Features

- STM32G474RE
- Up to 5 VL53L1X Time-of-Flight sensors
- Independent XSHUT control for each sensor
- Multiple I2C buses
- Automatic sequential sensor initialization
- Unique I2C address assignment
- Sensor online/offline status tracking
- Per-sensor distance and range status
- Initialization error detection
- Round-robin sensor reading

## Sensor Configuration

| Sensor | I2C | XSHUT | Assigned Address |
|---|---|---|---|
| TOF 1 | I2C1 | XSHUT1 | `0x54` |
| TOF 2 | I2C2 | XSHUT2 | `0x56` |
| TOF 3 | I2C2 | XSHUT2B | `0x58` |
| TOF 4 | I2C3 | XSHUT3 | `0x5A` |
| TOF 5 | I2C3 | XSHUT3B | `0x5C` |

Default VL53L1X address used by the ST API in this project:

```c
#define VL53_DEFAULT_ADDR 0x52

Main TOF configuration:
#define TOF_COUNT        5
#define TOF_READ_SLOT_MS 10

Current ranging configuration:
- Distance mode: Long
- Timing budget: 50 ms
- Inter-measurement period: 60 ms
- Read scheduler slot: 10 ms
Initialization Sequence
At startup, all VL53L1X sensors are first disabled through their XSHUT pins. The firmware then enables each sensor one at a time, waits for the boot process, verifies the sensor Model ID, configures the sensor, assigns a unique I2C address, and starts ranging.
This sequence allows multiple VL53L1X sensors to operate without address conflicts.
Initialization flow:
1. Disable all sensors through XSHUT
2. Enable one sensor
3. Wait for sensor boot
4. Verify Model ID
5. Initialize the VL53L1X
6. Configure distance mode
7. Configure timing budget
8. Configure inter-measurement period
9. Assign a unique I2C address
10. Start ranging
11. Mark the sensor as online
12. Continue with the next sensor
Sensor Data Structure
Each VL53L1X sensor is represented by the following structure:
typedef struct
{
    I2C_HandleTypeDef *i2c;

    GPIO_TypeDef *xshut_port;
    uint16_t xshut_pin;

    uint16_t addr;

    volatile uint16_t distance_mm;
    volatile uint8_t range_status;
    volatile uint8_t read_status;
    volatile uint8_t online;

} TOF_Sensor_t;

The sensor status and ranging data can be monitored directly during debugging.
Useful variables:
Variable	Description
tof[i].distance_mm	Measured distance in millimeters
tof[i].range_status	VL53L1X ranging status
tof[i].read_status	Communication or read status
tof[i].online	Indicates whether the sensor initialized successfully
tof_online_count	Number of sensors initialized successfully
tof_init_error	First initialization error detected
tof_current_index	Sensor currently being processed


Initialization Error Codes
The firmware uses different error ranges to identify the initialization stage where a sensor fails.
Error Code	Initialization Stage
10 - 14	Boot state communication error
20 - 24	Boot timeout
30 - 34	Model ID check failed
40 - 44	Sensor initialization failed
50 - 54	Distance mode configuration failed
60 - 64	Timing budget configuration failed
70 - 74	Inter-measurement configuration failed
80 - 84	I2C address assignment failed
90 - 94	Start ranging failed


The last digit represents the sensor index.
For example:
tof_init_error = 22

means sensor index 2 failed during the boot timeout stage.
Reading Method
Sensor measurements are processed using:
TOF_Read_One_By_One();

The firmware processes one sensor at a time.
For each sensor:
1. Select the corresponding I2C bus
2. Check whether new measurement data is ready
3. Read the ranging status
4. Read the measured distance
5. Store the result in tof[i].distance_mm
6. Clear the VL53L1X interrupt
7. Move to the next sensor
After all five sensors have been processed, the sensor index returns to the first sensor.
This keeps the sensor reading process simple and avoids blocking the main loop for all sensors at once.
Debugging
STM32CubeIDE Live Expressions can be used to monitor the TOF system in real time.
Recommended variables:
tof_online_count
tof_init_error
tof_current_index

tof[0].distance_mm
tof[1].distance_mm
tof[2].distance_mm
tof[3].distance_mm
tof[4].distance_mm

tof[0].online
tof[1].online
tof[2].online
tof[3].online
tof[4].online

For a correctly connected five-sensor setup:
tof_online_count = 5
tof_init_error   = 0

The distance values should change when an object is moved closer to or farther away from each sensor.
Development Environment
- STM32G474RE
- STM32CubeIDE
- STM32CubeMX
- STM32 HAL / LL drivers
- ST VL53L1X API
- ST-LINK for programming and debugging
CubeMX configuration file:
MainBoard_STM32G474RE_V01.ioc

Project Structure
STM32G474RE_VL53L1X
├── Core
│   ├── Inc
│   └── Src
├── Drivers
├── Middlewares
├── STM32CubeIDE
├── USB_Device
├── MainBoard_STM32G474RE_V01.ioc
└── README.md

The main VL53L1X initialization and reading logic is located in:
Core/Src/main.c

Notes
This repository is based on a larger STM32G474RE mainboard firmware and may contain drivers or source code for other peripherals.
The main focus of this repository is the implementation and testing of multiple VL53L1X Time-of-Flight sensors, including:
- Multi-sensor initialization
- XSHUT control
- Unique I2C address assignment
- Multi-I2C support
- Distance measurement
- Sensor status monitoring
- Error detection and debugging
