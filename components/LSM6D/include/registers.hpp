#ifndef _LSM_REGS_
#define _LSM_REGS_

#include <stdint.h>
#include "sdkconfig.h"

// This file contains all the registers of LSM6D as mentioned
// in the document at https://www.st.com/resource/en/datasheet/lsm6dsl.pdf

constexpr uint8_t WHO_AM_I = 0x0F; // Who am I?

// ------------ Control registers ------------- //

constexpr uint8_t CTRL1_XL = 0x10;
constexpr uint8_t CTRL2_G = 0x11; 
constexpr uint8_t CTRL3_C = 0x12;
constexpr uint8_t CTRL4_C = 0x13;
constexpr uint8_t CTRL5_C = 0x14;
constexpr uint8_t CTRL6_C = 0x15;
constexpr uint8_t CTRL7_G = 0x16;
constexpr uint8_t CTRL8_XL = 0x17; 
constexpr uint8_t CTRL9_XL = 0x18;
constexpr uint8_t CTRL10_C = 0x19;

// ------------ Temperature output ------------ //

constexpr uint8_t OUTX_TEMP_L = 0x20;
constexpr uint8_t OUTX_TEMP_H = 0x21;

// ------------ Gyroscope output ------------ //

constexpr uint8_t OUTX_L_G = 0x22;
constexpr uint8_t OUTX_H_G = 0x23;
constexpr uint8_t OUTY_L_G = 0x24;
constexpr uint8_t OUTY_H_G = 0x25;
constexpr uint8_t OUTZ_L_G = 0x26;
constexpr uint8_t OUTZ_H_G = 0x27;

// ---------  Accelerometer output ---------- //

constexpr uint8_t OUTX_L_XL = 0x28;
constexpr uint8_t OUTX_H_XL = 0x29;
constexpr uint8_t OUTY_L_XL = 0x2A;
constexpr uint8_t OUTY_H_XL = 0x2B;
constexpr uint8_t OUTZ_L_XL = 0x2C;
constexpr uint8_t OUTZ_H_XL = 0x2D;

// ------------  FIFO registers ------------- //

constexpr uint8_t FIFO_STATUS1 = 0x3A;
constexpr uint8_t FIFO_STATUS2 = 0x3B;
constexpr uint8_t FIFO_STATUS3 = 0x3C;
constexpr uint8_t FIFO_STATUS4 = 0x3D;
constexpr uint8_t FIFO_DATA_OUT_L = 0x3E; 
constexpr uint8_t FIFO_DATA_OUT_H = 0x3F;

// ------------------------------------------ //

#endif