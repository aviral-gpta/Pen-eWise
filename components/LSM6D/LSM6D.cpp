#include "LSM6D.hpp"
#include "include/registers.hpp"
#include "include/fifo.hpp"

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
        LSM_ERR_CHECK(XL_off());
        LSM_ERR_CHECK(G_off());
        return err;
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

    // Accelerometer functions

    esp_err_t LSM::XL_init(uint8_t mode){
        LSM_ERR_CHECK(writeBit(INT1_CTRL, 0, 0x01));
        LSM_ERR_CHECK(writeBits(CTRL1_XL, 4, 4, mode));
        uint8_t val;
        readByte(CTRL1_XL, &val);
        ESP_LOGI("XL mode", "%i", val);
        return err;
    }

    esp_err_t LSM::XL_off(){
        LSM_ERR_CHECK(writeBit(INT1_CTRL, 0, 0x00));
        LSM_ERR_CHECK(writeByte(CTRL1_XL, 0x00));
        return 0;
    }   

    // Gyroscope helpers

    esp_err_t LSM::G_init(uint8_t mode){
        LSM_ERR_CHECK(writeBit(INT1_CTRL, 1, 0x01));
        LSM_ERR_CHECK(writeBits(CTRL2_G, 4, 4, mode));
        return err;
    }

    esp_err_t LSM::G_off(){
        LSM_ERR_CHECK(writeBit(INT1_CTRL, 1, 0x00));
        LSM_ERR_CHECK(writeByte(CTRL2_G, 0x00));
        return err;
    }   

    // FIFO 

    // esp_err_t LSM::FIFOStatus(uint16_t *val){

    // }

    esp_err_t LSM::FIFO_init(lsm_fifo_mode_t mode, uint8_t XL_ODR, uint8_t G_ODR, uint16_t wtm_level){
        // Set accelerometer and gyroscope BDR (<= their ODR)
        LSM_ERR_CHECK(writeByte(FIFO_CTRL3, XL_ODR | (G_ODR << 4)));
        // set FIFO mode
        LSM_ERR_CHECK(writeByte(FIFO_CTRL4, 0x06));
        // Generate DEN signal
        writeByte(CTRL9_XL, 0xE8);
        writeByte(CTRL6_C, 0xC0);
        LSM_ERR_CHECK(writeByte(CTRL1_XL, (XL_ODR << 4) | 0x0C));
        LSM_ERR_CHECK(writeByte(CTRL2_G, (G_ODR << 4) | 0x0C));
        return err;
    }

    esp_err_t LSM::readFIFO(){
        uint8_t lo, hi;
        readByte(FIFO_STATUS1, &lo);
        readByte(FIFO_STATUS2, &hi);
        uint16_t numUnread = uint16_t(lo) + (uint16_t(hi & 03) << 8);
        struct fifo_data_t* dataArray = (fifo_data_t*)(malloc(numUnread * sizeof(struct fifo_data_t)));
        for(int i = 0; i < numUnread; ++i){
            uint8_t tg = 0;
            uint8_t msk = ((1 << 5) - 1) << 3;
            LSM_ERR_CHECK(readByte(FIFO_DATA_OUT_TAG, &tg));
            tg &= msk;
            tg = tg >> 3;
            dataArray[i].tag = static_cast<lsm_fifo_tag_t>(tg);
            uint8_t data_x[2], data_y[2], data_z[2]; 
            LSM_ERR_CHECK(readBytes(FIFO_DATA_OUT_X_L, 2, data_x));
            LSM_ERR_CHECK(readBytes(FIFO_DATA_OUT_Y_L, 2, data_y));
            LSM_ERR_CHECK(readBytes(FIFO_DATA_OUT_Z_L, 2, data_z));
            dataArray[i].data_x = (uint16_t)data_x[0] | (((uint16_t)data_x[1]) << 8);
            dataArray[i].data_y = (uint16_t)data_y[0] | (((uint16_t)data_y[1]) << 8);
            dataArray[i].data_z = (uint16_t)data_z[0] | (((uint16_t)data_z[1]) << 8);
        }
        free(dataArray);
        return err;
    }

    esp_err_t LSM::setWatermark(uint16_t wtm){
        uint8_t wtm_val[2] = {static_cast<uint8_t>(wtm & 0xFF), static_cast<uint8_t>(wtm >> 8)};
        LSM_ERR_CHECK(writeByte(FIFO_CTRL1, wtm_val[0]));
        LSM_ERR_CHECK(writeBit(FIFO_CTRL2, 0, wtm_val[1] & 1));
        return err;
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
        printXL(curr1, curr2, 2);
    }

    void LSM::printXL(uint8_t lo, uint8_t hi, uint8_t scale){
        int16_t x = 0x0;
        x = x | hi;
        x = x << 8;
        x = x | lo;
        printXL(x, scale);
        return;
    }

    void LSM::printXL(uint16_t raw_val, uint8_t scale){
        float val = (float(raw_val) / float(1 << 16)) * scale;
        ESP_LOGI("LSM", "Accelerometer reading: %f ", val);
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