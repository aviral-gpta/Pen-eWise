#ifndef _LSM6D_FIFO_
#define _LSM6D_FIFO_

typedef enum : uint8_t{
    G_NC = 0x01,            // Gyroscope non-compressed
    XL_NC = 0x02,           // Accelerometer non-compressed
    TEMP = 0x03,            // Temperature
    TIME = 0x04,            // Time
    CFG_CNG = 0x05,         // Configuration change
    XL_NC_T_2 = 0x06,       // XL_NC at batched at 2x last time slot
    XL_NC_T_1 = 0x07,       // XL_NC at batched at last time slot
    XL_2XC = 0x08,
    XL_3XC = 0x09,
    G_NC_T_2 = 0x0A,
    G_NC_T_1 = 0x0B,
    G_2XC = 0x0C,
    G_3XC = 0x0D,
    SNS_0 = 0x0E,
    SNS_1 = 0x0F,
    SNS_2 = 0x10,
    SNS_3 = 0x11,
    STP_CNTR = 0x12,
    SNS_NACK = 0x19
} lsm_fifo_tag_t;

struct fifo_data_t {
    // Generic fifo data type
    lsm_fifo_tag_t tag;
    uint16_t data_x;
    uint16_t data_y;
    uint16_t data_z;
};

#endif