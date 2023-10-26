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
// ...

#define PORT 3333

gpio_num_t SDA = GPIO_NUM_10;
gpio_num_t SCL = GPIO_NUM_8;
uint32_t CLOCK_SPEED = 400000;
MPU_t MPU;
const int chunkSize = 50;
constexpr uint16_t kFIFOPacketSize = 12*chunkSize;
uint8_t buffer[kFIFOPacketSize];
float roll{0}, pitch{0}, yaw{0};
int16_t rawData[chunkSize][6] = {0};
bool is_connected = false;
int sockNum=-1;

static const char* MPUTAG = "MPU";
static const char* TCPTAG = "TCP";
void mpuTask(void*);
void TCP_transmit();

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
    i2c0.begin(SDA, SCL, CLOCK_SPEED);
    mpu.setBus(i2c0);
    mpu.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);
    while (esp_err_t err = mpu.testConnection()) {
        ESP_LOGE(MPUTAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(MPUTAG, "MPU connection successful!");
    ESP_ERROR_CHECK(mpu.initialize());
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

extern "C" void app_main() {
vTaskDelay(1000 / portTICK_PERIOD_MS);
    MPU_init(MPU);
    calibrate_IMU(MPU);

    MPU.setSampleRate(100U);
    MPU.setAccelFullScale(mpud::ACCEL_FS_2G);
    MPU.setGyroFullScale(mpud::GYRO_FS_250DPS);
    MPU.setDigitalLowPassFilter(mpud::DLPF_42HZ);  
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // MPU.compassInit();
    
    // example_TCP_init();
    // TCP_connect();

    ESP_ERROR_CHECK(MPU.setFIFOConfig(mpud::FIFO_CFG_ACCEL | mpud::FIFO_CFG_GYRO));
    ESP_ERROR_CHECK(MPU.setFIFOEnabled(true));
    ESP_ERROR_CHECK(MPU.resetFIFO());
    xTaskCreate(mpuTask, "MPUTask", 2 * 1024, nullptr, 5, nullptr);
    

    // MPU.acceleration(&accelRaw);  // fetch raw data from the registers
    // MPU.rotation(&gyroRaw);       // fetch raw data from the registers
    // printf("accel: %+d %+d %+d\n", accelRaw.x, accelRaw.y, accelRaw.z);
    // printf("gyro: %+d %+d %+d\n", gyroRaw[0], gyroRaw[1], gyroRaw[2]);
    // mpud::float_axes_t accelG = mpud::accelms2(accelRaw, mpud::ACCEL_FS_4G);  // raw data to gravity
    // mpud::float_axes_t gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);  // raw data to º/s
    // printf("accel: %+.2f %+.2f %+.2f %+.2f %+.2f %+.2f\n", accelG.x, accelG.y, accelG.z, gyroDPS.x, gyroDPS.y, gyroDPS.z);
    // printf("gyro: %+.2f %+.2f %+.2f\n", gyroDPS.x, gyroDPS.y, gyroDPS.z);
    // vTaskDelay(200 / portTICK_PERIOD_MS);

    }


void TCP_transmit(){    
    if (sockNum < 0) {
        ESP_LOGE(TCPTAG, "Socket Error: errno %d", errno);
        // break;
    }
    int err = send(sockNum, &rawData, sizeof(rawData), 0);
    if (err < 0) {
        ESP_LOGE(TCPTAG, "Error occurred during sending: errno %d", errno);
        // break;
    }
    else if(err == 0){
        ESP_LOGE(TCPTAG, "Data sent");
        // break;
    }
}

void mpuTask(void*){
    while(1){
        uint16_t fifocount = MPU.getFIFOCount();
        // ESP_LOGI(TAG, "FIFO count: %d", fifocount);
        if(fifocount>= kFIFOPacketSize) {
            
            MPU.readFIFO(kFIFOPacketSize, buffer);
            for(int i=0; i<chunkSize; i++){
                mpud::raw_axes_t rawAccel, rawGyro;
                rawAccel.x = buffer[0 + i*12] << 8 | buffer[1 + i*12];
                rawAccel.y = buffer[2 + i*12] << 8 | buffer[3 + i*12];
                rawAccel.z = buffer[4 + i*12] << 8 | buffer[5 + i*12];
                rawGyro.x  = buffer[6 + i*12] << 8 | buffer[7 + i*12];
                rawGyro.y  = buffer[8 + i*12] << 8 | buffer[9 + i*12];
                rawGyro.z  = buffer[10 + i*12] << 8 | buffer[11 + i*12];
                rawData[i][0] = htons(rawAccel.x);
                rawData[i][1] = htons(rawAccel.y);
                rawData[i][2] = htons(rawAccel.z);
                rawData[i][3] = htons(rawGyro.x);
                rawData[i][4] = htons(rawGyro.y);
                rawData[i][5] = htons(rawGyro.z);
                constexpr double kRadToDeg = 57.2957795131;
                constexpr float kDeltaTime = 1.f / 100;
                float gyroRoll             = roll + mpud::math::gyroDegPerSec(rawGyro.x, mpud::GYRO_FS_250DPS) * kDeltaTime;
                float gyroPitch            = pitch + mpud::math::gyroDegPerSec(rawGyro.y, mpud::GYRO_FS_250DPS) * kDeltaTime;
                float gyroYaw              = yaw + mpud::math::gyroDegPerSec(rawGyro.z, mpud::GYRO_FS_250DPS) * kDeltaTime;
                float accelRoll            = atan2(-rawAccel.x, rawAccel.z) * kRadToDeg;
                float accelPitch = atan2(rawAccel.y, sqrt(rawAccel.x * rawAccel.x + rawAccel.z * rawAccel.z)) * kRadToDeg;
                // Fusion
                roll  = gyroRoll * 0.95f + accelRoll * 0.05f;
                pitch = gyroPitch * 0.95f + accelPitch * 0.05f;
                yaw   = gyroYaw;
                // correct yaw
                if (yaw > 180.f)
                    yaw -= 360.f;
                else if (yaw < -180.f)
                    yaw += 360.f;
                printf("Pitch: %+6.1f \t Roll: %+6.1f \t Yaw: %+6.1f \n", pitch, roll, yaw);
            }
            // TCP_transmit();
            
        }
    }
}