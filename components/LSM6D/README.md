# LSM6D

# LSM6D Documentation

## Overview

The LSM6D is a sensor module designed to measure acceleration and gyroscope data. This documentation provides an overview of the LSM6D class and its functions.

## Class: LSM

### Constructors

- **LSM(lsm_bus_t& bus)**:
  - Initializes the LSM sensor with the provided bus interface.
  
- **LSM(lsm_bus_t& bus, lsm_i2caddr_t addr)**:
  - Initializes the LSM sensor with the provided bus interface and I2C address.

### Methods

- **esp_err_t close()**:
  - Disables both the accelerometer and gyroscope sensors.
  
- **lsm_bus_t& getBus()**:
  - Returns the bus interface used by the LSM sensor.
  
- **esp_err_t setBus(lsm_bus_t &bus)**:
  - Sets the bus interface for communication with the LSM sensor.
  
- **lsm_i2caddr_t getAddr()**:
  - Returns the I2C address of the LSM sensor.
  
- **esp_err_t setAddr(lsm_i2caddr_t addr)**:
  - Sets the I2C address of the LSM sensor.
  
- **esp_err_t XL_init(uint8_t mode)**:
  - Initializes the accelerometer with the specified mode.
  
- **esp_err_t XL_off()**:
  - Turns off the accelerometer.
  
- **esp_err_t G_init(uint8_t mode)**:
  - Initializes the gyroscope with the specified mode.
  
- **esp_err_t G_off()**:
  - Turns off the gyroscope.

## FIFO

- **esp_err_t FIFO_init(lsm_fifo_mode_t mode, uint8_t XL_ODR, uint8_t G_ODR, uint16_t wtm_level)**

    - Initializes the FIFO feature of the LSM sensor.

    #### Parameters:

    - `mode`: The FIFO mode to be set.
    - `XL_ODR`: Output Data Rate (ODR) for the accelerometer.
    - `G_ODR`: Output Data Rate (ODR) for the gyroscope.
    - `wtm_level`: Water-mark level specifying the number of samples to trigger an interrupt when the FIFO buffer exceeds this level.

    #### Return Value:

    Returns an `esp_err_t` indicating the success or failure of the FIFO initialization.

    #### Example Usage:

    ```cpp
    LSM sensor;
    esp_err_t result = sensor.FIFO_init(FIFO_MODE_XL_G, ODR_100_Hz, ODR_100_Hz, 32);
    if (result != ESP_OK) {
        // Handle error
    }
    ```

- **esp_err_t setWatermark(uint16_t wtm)**:
  - Sets the watermark level for the FIFO.
