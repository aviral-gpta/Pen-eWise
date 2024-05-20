#ifndef _LSM6D_
#define _LSM6D_

#include "I2Cbus.hpp"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"

#define LSM_ERR_CHECK(x) LSM6D::errorCheckLogger(x, __FUNCTION__, __LINE__, #x)

typedef I2C_t lsm_bus_t;

const int8_t bootTime = 15;                                 // 15ms to boot LSM6D - for reference

typedef enum : uint8_t{
    LSM6D_I2C_ADD_LOW = 0X6A,                               // SD0/SA0 LOW
    LSM6D_I2C_ADD_HIGH = 0X6B                                // SD0/SA0 HIGH
} lsm_i2caddr_t;

typedef enum : uint8_t{ 
    LSM_FIFO_OFF = 0X0,                                     // FIFO OFF
    LSM_FIFO_DEF = 0X1,                                     // Fill until FIFO full
    LSM_FIFO_CONT_TRIG = 0X3,                               // Continuous until triggered to FIFO
    LSM_FIFO_BY_TRIG = 0X4,                                 // Bypass until trigerred to FIFO
    LSM_FIFO_CONT = 0X6                                     // Continuous mode - Write on older data if FIFO fills
} lsm_fifo_mode_t;

namespace LSM6D{

    static esp_err_t err;                           // Last error

    inline esp_err_t errorCheckLogger(esp_err_t x, const char* func, const int line, const char* expr) {
        // Error logger function
        // Default level -> Error
        if (x) ESP_LOGE("LSM6D", "func:%s @ line:%d, expr:\"%s\", error:0x%X", func, line, expr, x);
        err = x;
        return x;
    }

    class LSM{

        private:
            lsm_bus_t* bus;                                 // Points to the I2C bus
            lsm_i2caddr_t addr;                             // Slave address
            uint8_t buffer[16];                             // Common buffer for temporary data
            bool fifoValid[4][2];
            uint8_t fifoTemp[4][2];
        public:


            // Constructors and Destructors

            LSM() = default;
            // ~LSM();                                         // Default constuctor and destructor, use compiler generated versions
            LSM(lsm_bus_t& bus);                            // Specify the bus, don't convert to I2C_t
            LSM(lsm_bus_t& bus, lsm_i2caddr_t addr);

            esp_err_t close();                              // call before destructing

            // getters and setters

            lsm_bus_t& getBus();    
            esp_err_t setBus(lsm_bus_t &bus);
            lsm_i2caddr_t getAddr();
            esp_err_t setAddr(lsm_i2caddr_t addr);

            // Accelerometer functions

            esp_err_t XL_init(uint8_t mode = 0x6);
            esp_err_t XL_off();
            esp_err_t setAccelFullScale(uint8_t scale);
            esp_err_t setAccelODR(uint8_t odr);
            bool acc_available();

            // Gyroscope functions
            esp_err_t setGyroFullScale(uint8_t scale);
            esp_err_t setGyroODR(uint8_t odr);
            esp_err_t G_init(uint8_t mode = 0x6);
            esp_err_t G_off();
            bool gyro_available();

            // FIFO functions
            uint16_t getFIFOCount();
            esp_err_t FIFO_init(lsm_fifo_mode_t mode, uint8_t XL_ODR, uint8_t G_ODR, uint16_t wtm_level = 200);
            esp_err_t readFIFO(uint16_t length, uint8_t* data);              // Read Data(2 Byte long) to 'data'
            esp_err_t setWatermark(uint16_t wtm = 1 << 5);

            // Utils

            esp_err_t reset();
            esp_err_t testConnection();                     // test connection by reading who_am_i register

            void getAcc();
            void printXL(uint8_t lo, uint8_t hi, uint8_t scale);
            void printXL(int16_t x_val, int16_t y_val, int16_t z_val, uint8_t scale, uint8_t time);

            void printgyro(int16_t x_val, int16_t y_val, int16_t z_val, uint8_t scale, uint8_t time);

            // Tap detection

            // Reading and writing functions

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