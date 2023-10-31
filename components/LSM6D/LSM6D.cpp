#include "LSM6D.hpp"
#include "include/registers.hpp"

namespace LSM6D{
    LSM::~LSM(){
        close();
    }
    LSM::LSM(lsm_bus_t& bus){
        this->bus = &bus;
    }
    LSM::LSM(lsm_bus_t& bus, lsm_i2caddr_t addr){
        this->bus = &bus;
        this->addr = addr;
    }
    esp_err_t LSM::close(){
        writeByte(CTRL1_XL, 0x00);
        writeByte(CTRL2_G, 0x00);
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
    
    esp_err_t LSM::testConnection(){
        uint8_t val;
        readByte(WHO_AM_I, &val);
        return val == 0x6A ? 0 : -1;
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