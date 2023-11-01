#ifndef _LSM6D_
#define _LSM6D_

#include "I2Cbus.hpp"

typedef I2C_t lsm_bus_t;

typedef enum : uint8_t{
    LSM6D_I2C_ADD_LOW = 0X6A,                               // SD0/SA0 LOW
    LSM6D_I2C_ADD_HIGH = 0X6B                                // SD0/SA0 HIGH
} lsm_i2caddr_t;

typedef enum : uint8_t{ 
    LSM_FIFO_OFF = 0X0,                                     // FIFO OFF
    LSM_FIFO_DEF = 0X1,                                     // Fill until FIFO full
    LSM_FIFO_CONT_TRIG = 0X3,                               // Continuous until triggered to FIFO
    LSM_FIFO_BY_TRIG = 0X4,                                 // Bypass until trigerred to FIFO
    LSM_FIFO_CONT = 0X6                                     // Continuous mode - Write on older if data fills
} lsm_fifo_mode_t;

namespace LSM6D{
    class LSM{
        private:
            lsm_bus_t* bus;                                 // Points to the I2C bus
            lsm_i2caddr_t addr;                             // Slave address
            uint8_t buffer[16];
            esp_err_t err;                                  // Use for error handling 
        public:
            // Constructors and Destructors

            LSM() = default;
            ~LSM();                                         // Default constuctor and destructor, use compiler generated versions
            LSM(lsm_bus_t& bus);                            // Specify the bus, don't convert to I2C_t
            LSM(lsm_bus_t& bus, lsm_i2caddr_t addr);

            esp_err_t close();                              // call before destructing

            // getters and setters

            lsm_bus_t& getBus();    
            esp_err_t setBus(lsm_bus_t &bus);
            lsm_i2caddr_t getAddr();
            esp_err_t setAddr(lsm_i2caddr_t addr);

            // Accelerometer helpers

            inline esp_err_t acc_on(uint8_t mode);
            inline esp_err_t acc_off();
            bool acc_available();

            // Gyroscope helpers

            inline esp_err_t gyro_on(uint8_t mode);
            inline esp_err_t gyro_off();
            bool gyro_available();

            // FIFO functions

            esp_err_t setFIFOMode(lsm_fifo_mode_t mode);
            bool isBelowMark();
            bool isFilled();
            bool isEmpty();
            bool isFullSmart();
            uint16_t numUnread();
            esp_err_t readFIFO(uint16_t* data);              // Read Data(2 Byte long) to 'data'

            // Utils

            esp_err_t testConnection();                     // test connection by reading who_am_i register

            esp_err_t readBit(uint8_t regAddr, uint8_t bitNum, uint8_t* data);
            esp_err_t readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);
            esp_err_t readByte(uint8_t regAddr, uint8_t* data);
            esp_err_t readBytes(uint8_t regAddr, size_t length, uint8_t* data);

            esp_err_t writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
            esp_err_t writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
            esp_err_t writeByte(uint8_t regAddr, uint8_t data);
            esp_err_t writeBytes(uint8_t regAddr, size_t length, const uint8_t* data);
    };
}

#endif