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

    // Accelerometer helpers

    inline esp_err_t LSM::acc_on(uint8_t mode){
        // writeBit(INT1_CTRL, 0, 0);
        return writeByte(CTRL1_XL, mode);
    }

    inline esp_err_t LSM::acc_off(){
        writeByte(CTRL1_XL, 0x00);
        return 0;
    }   

    // Gyroscope helpers

    inline esp_err_t LSM::gyro_on(uint8_t mode){
        // writeBit(INT1_CTRL, 1, 0);
        return writeByte(CTRL2_G, mode);
    }

    inline esp_err_t LSM::gyro_off(){
        writeByte(CTRL1_XL, 0x00);
        return 0;
    }   

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
        ESP_LOGI("LSM", "Read %u from WHO_AM_I", val);
        return val == 64 ? ESP_OK : ESP_ERR_NOT_FOUND;
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