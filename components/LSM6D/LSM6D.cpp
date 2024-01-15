#include "LSM6D.hpp"
#include "include/registers.hpp"

namespace LSM6D{
    // LSM::~LSM(){
    //     close();
    // }
    LSM::LSM(lsm_bus_t& bus){
        this->bus = &bus;
    }
    LSM::LSM(lsm_bus_t& bus, lsm_i2caddr_t addr){
        this->bus = &bus;
        this->addr = addr;
    }
    esp_err_t LSM::close(){
        if(!(acc_off() != 0 || gyro_off() != 0)){
            return -1;
        }
        return 0;
    }

    lsm_bus_t& LSM::getBus(){
        return *(this->bus);
    }    
    esp_err_t LSM::setBus(lsm_bus_t &bus){
        this->bus = &bus;
        return 0;
    }
    lsm_i2caddr_t LSM::getAddr(){
        return this->addr;
    }
    esp_err_t LSM::setAddr(lsm_i2caddr_t addr){
        this->addr = addr;
        return 0;
    }

    // uint8_t LMS::readStatus(){
    //     uint8_t val;
    //     readByte(STATUS_REG, &val);
    //     return val;
    // }

    // Accelerometer helpers

    // inline esp_err_t LSM::acc_on(uint8_t mode = 0x60){
    //     writeBit(INT1_CTRL, 0, 1);
    //     return writeByte(CTRL1_XL, mode);
    // }

    // inline esp_err_t LSM::acc_off(){
    //     writeBit(INT1_CTRL, 0, 0);
    //     writeByte(CTRL1_XL, 0x00);
    //     return 0;
    // }   

    // Gyroscope helpers

    // inline esp_err_t LSM::gyro_on(uint8_t mode - 0x60){
    //     writeBit(INT1_CTRL, 1, 1);
    //     return writeByte(CTRL2_G, mode);
    // }

    // inline esp_err_t LSM::gyro_off(){
    //     writeByte(CTRL1_XL, 0x00);
    //     return 0;
    // }   

    // FIFO options

    esp_err_t LSM::setFIFOMode(lsm_fifo_mode_t mode){
        return writeBits(FIFO_CTRL5, 0, 3, mode);
    }

    bool LSM::isBelowMark(){
        uint8_t val = 0;
        readBit(FIFO_STATUS2, 7, &val);
        return (val == 0) ? true : false;
    }

    bool LSM::isFilled(){
        uint8_t val = 0;
        readBit(FIFO_STATUS2, 6, &val);
        return (val == 0) ? false : true;
    }

    bool LSM::isEmpty(){
        uint8_t val = 0;
        readBit(FIFO_STATUS2, 4, &val);
        return (val == 0) ? true : false;
    }

    bool LSM::isFullSmart(){
        uint8_t val = 0;
        readBit(FIFO_STATUS2, 5, &val);
        return (val == 0) ? false : true;
    }

    uint16_t LSM::numUnread(){
        u_int8_t low = 0, high = 0;
        readByte(FIFO_STATUS1, &low);
        readBits(FIFO_STATUS2, 0, 3, &high);
        uint16_t unread = 0;         
        unread = unread or high;
        unread = (unread << 8);
        unread = unread or low;
        return unread;
    }

    esp_err_t LSM::readFIFO(uint16_t* data){
        u_int8_t low = 0, high = 0;
        readByte(FIFO_DATA_OUT_L, &low);
        readByte(FIFO_DATA_OUT_H, &high);
        uint16_t val = 0;            
        val = val or high;
        val = (val << 8);
        val = val or low;
        *data = val;
        return 0;
    }

    // Utils

    esp_err_t LSM::testConnection(){
        uint8_t val;
        readByte(WHO_AM_I, &val);
        return val == 0x06B ? ESP_OK : ESP_ERR_NOT_FOUND;
    }

    void LSM::getAcc(){
        uint8_t x = (uint8_t)0x10;
        uint8_t y = (uint8_t)0x60;
        writeByte(x, y);
        uint8_t curr1, curr2;
        readByte(OUTZ_L_XL, &curr1);
        readByte( 0x2D, &curr2);
        printAcc(curr1, curr2);
    }

    void LSM::printAcc(uint8_t lo, uint8_t hi, uint8_t scale){
        int16_t x = 0x0;
        x = x | hi;
        x = x << 8;
        x = x | lo;
        int mx = 1 << 15;
        float val = float(x) / float(mx);
        val *= scale;
        ESP_LOGI("LSM", "Accelerometer reading: %f ", val);
        return;
    }

    esp_err_t LSM::reset(){
        return ESP_OK;
    }

    /*! Read a single bit from a register*/
    inline esp_err_t LSM::readBit(uint8_t regAddr, uint8_t bitNum, uint8_t* data)
    {
        return err = bus->readBit(addr, regAddr, bitNum, data);
    }
    /*! Read a range of bits from a register */
    inline esp_err_t LSM::readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data)
    {
        return err = bus->readBits(addr, regAddr, bitStart, length, data);
    }
    /*! Read a single register */
    inline esp_err_t LSM::readByte(uint8_t regAddr, uint8_t* data)
    {
        return err = bus->readByte(addr, regAddr, data);
    }
    /*! Read data from sequence of registers */
    inline esp_err_t LSM::readBytes(uint8_t regAddr, size_t length, uint8_t* data)
    {
        return err = bus->readBytes(addr, regAddr, length, data);
    }
    /*! Write a single bit to a register */
    inline esp_err_t LSM::writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data)
    {
        return err = bus->writeBit(addr, regAddr, bitNum, data);
    }
    /*! Write a range of bits to a register */
    inline esp_err_t LSM::writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
    {
        return err = bus->writeBits(addr, regAddr, bitStart, length, data);
    }
    /*! Write a value to a register */
    inline esp_err_t LSM::writeByte(uint8_t regAddr, uint8_t data)
    {
        return err = bus->writeByte(addr, regAddr, data);
    }
    /*! Write a sequence to data to a sequence of registers */
    inline esp_err_t LSM::writeBytes(uint8_t regAddr, size_t length, const uint8_t* data)
    {
        return err = bus->writeBytes(addr, regAddr, length, data);
    }
}