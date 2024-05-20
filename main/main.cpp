#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "MPU.hpp"        // main file, provides the class itself
#include "mpu/math.hpp"   // math helper for dealing with MPU data
#include "mpu/types.hpp"  // MPU data types and definitions
#include "I2Cbus.hpp"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_event.h"
#include <sys/socket.h>
#include <netdb.h>

#include "LSM6D.hpp"
// ...

#define PORT 3333

gpio_num_t SDA = GPIO_NUM_10;
gpio_num_t SCL = GPIO_NUM_8;
uint32_t CLOCK_SPEED = 400000;
MPU_t MPU;
const int chunkSize = 100;
constexpr uint16_t MPUPacketSize = 12*chunkSize;
constexpr uint16_t LSMPacketSize = 14*chunkSize;
uint8_t MPUbuffer[MPUPacketSize];
uint8_t LSMbuffer[LSMPacketSize]={0};
// const int LSM_chunkSize = 200;
// struct fifo_data_t* LSMdata = (fifo_data_t*)(malloc(length * sizeof(struct fifo_data_t)))
float roll{0}, pitch{0}, yaw{0};
int16_t rawData[chunkSize][6] = {0};
bool is_connected = false;
int sockNum=-1;

static const char* MPUTAG = "MPU";
static const char* TCPTAG = "TCP";
void mpuTask(void*);
void lsmTask(void*);
void TCP_transmit(const void *dataptr, size_t size);

void calibrate_IMU(MPU_t mpu){
    mpud::selftest_t retSelfTest;
    while (esp_err_t err = mpu.selfTest(&retSelfTest)) {
        ESP_LOGE(MPUTAG, "Failed to perform MPU Self-Test, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(MPUTAG, "MPU Self-Test result: Gyro=%s Accel=%s",  //
             (retSelfTest & mpud::SELF_TEST_GYRO_FAIL ? "FAIL" : "OK"),
             (retSelfTest & mpud::SELF_TEST_ACCEL_FAIL ? "FAIL" : "OK"));
    mpud::raw_axes_t accelBias, gyroBias;
    ESP_ERROR_CHECK(mpu.computeOffsets(&accelBias, &gyroBias));
    ESP_ERROR_CHECK(mpu.setAccelOffset(accelBias));
    ESP_ERROR_CHECK(mpu.setGyroOffset(gyroBias));
}

void MPU_init(MPU_t mpu){
    // i2c0.begin(SDA, SCL, CLOCK_SPEED);
    mpu.setBus(i2c0);
    mpu.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
    while (esp_err_t err = mpu.testConnection()) {
        ESP_LOGE(MPUTAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(MPUTAG, "MPU connection successful!");
    ESP_ERROR_CHECK(mpu.initialize());
}

void LSM_init(LSM6D::LSM *lsm){
    i2c0.begin(SDA, SCL, CLOCK_SPEED);
    lsm->setBus(i2c0);
    lsm->setAddr(LSM6D_I2C_ADD_LOW);
    while (esp_err_t err = lsm->testConnection()) {
        ESP_LOGE("LSM", "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI("LSM", "LSM connection successful!");
    // ESP_ERROR_CHECK(mpu.initialize());
}

void example_TCP_init(){
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(example_connect());
}

void TCP_connect(){
    char host_ip[] = "172.20.10.2";
    int addr_family = 0;
    int ip_protocol = 0;
    struct sockaddr_in dest_addr;
    inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TCPTAG, "Unable to create socket: errno %d", errno);
        // break;
    }
    ESP_LOGI(TCPTAG, "Socket created, connecting to %s:%d", host_ip, PORT);

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    while(err != 0) {
        ESP_LOGE(TCPTAG, "Socket unable to connect: errno %d", errno);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    }
    is_connected = true;
    ESP_LOGI(TCPTAG, "Successfully connected");
    sockNum =  sock;
}
LSM6D::LSM lsmtest;

extern "C" void app_main() {

    gpio_pulldown_en(GPIO_NUM_20);
    gpio_set_direction(GPIO_NUM_20, GPIO_MODE_OUTPUT);
    example_TCP_init();
    TCP_connect();
    LSM_init(&lsmtest);
    lsmtest.setAccelFullScale(0x00);
    lsmtest.setGyroFullScale(0x00);

    gpio_set_direction(GPIO_NUM_20, GPIO_MODE_INPUT); 
    lsmtest.FIFO_init(LSM_FIFO_CONT, 0x04, 0x04, 200U);  //WTM level should be set according to the chunk size. 1LSB of WTM is 7 bytes.
    
    MPU_init(MPU);
    calibrate_IMU(MPU);

    MPU.setSampleRate(100U);
    MPU.setAccelFullScale(mpud::ACCEL_FS_2G);
    MPU.setGyroFullScale(mpud::GYRO_FS_250DPS);
    MPU.setDigitalLowPassFilter(mpud::DLPF_42HZ);  
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    MPU.compassInit();

    ESP_ERROR_CHECK(MPU.setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO));
    ESP_ERROR_CHECK(MPU.setFIFOEnabled(true));
    ESP_ERROR_CHECK(MPU.resetFIFO());
    xTaskCreate(mpuTask, "MPUTask", 2 * 1024, nullptr, 5, nullptr);
    xTaskCreate(lsmTask, "LSMTask", 2 * 1024, nullptr, 5, nullptr);
    }


void TCP_transmit(const void *dataptr, size_t size){    
    if (sockNum < 0) {
        ESP_LOGE(TCPTAG, "Socket Error: errno %d", errno);
        // break;
    }
    int err = send(sockNum, dataptr, size, 0);
    if (err < 0) {
        ESP_LOGE(TCPTAG, "Error occurred during sending: errno %d", errno);
        // break;
    }
    else if(err >= 0){
        ESP_LOGI(TCPTAG, "Data sent");
        // break;
    }
}

void lsmTask(void*){
    while(1){
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if(gpio_get_level(GPIO_NUM_20)){
            uint16_t cnt=lsmtest.getFIFOCount();
            // ESP_LOGI("LSM", "FIFO count: %d", cnt);
            lsmtest.readFIFO(chunkSize, LSMbuffer);
            TCP_transmit(&LSMbuffer, sizeof(LSMbuffer));
        }
    }
}

void mpuTask(void*){
    while(1){
        vTaskDelay(10 / portTICK_PERIOD_MS);
        uint16_t fifocount = MPU.getFIFOCount();
        // ESP_LOGI(TAG, "FIFO count: %d", fifocount);
        if(fifocount>= MPUPacketSize) {
            MPU.readFIFO(MPUPacketSize, MPUbuffer);
            TCP_transmit(&MPUbuffer, sizeof(MPUbuffer));
        }
    }
}