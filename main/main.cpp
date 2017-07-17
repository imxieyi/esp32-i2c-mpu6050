#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "mpu6050.hpp"
#include <cmath>

static void mpu6050_task(void *pvParameters) {
    MPU6050 mpu(GPIO_NUM_19, GPIO_NUM_22, I2C_NUM_0);

    if(!mpu.init()) {
	    ESP_LOGE("mpu6050", "init failed!");
        vTaskDelete(0);
    }
	ESP_LOGI("mpu6050", "init success!");

    short ax,ay,az,gx,gy,gz,t;
    float accx,accy,accz;
    float pitch, roll;

    while(1) {
        ax = -mpu.getAccX();
        ay = -mpu.getAccY();
        az = -mpu.getAccZ();
        gx = mpu.getGyroX();
        gy = mpu.getGyroY();
        gz = mpu.getGyroZ();
        accx = ax;
        accy = ay;
        accz = az;
        pitch = atan(accx/accz)*57.2958;
        roll = atan(accy/accz)*57.2958;
        ESP_LOGI("mpu6050", "Acc: ( %d, %d, %d)", ax, ay, az);
        ESP_LOGI("mpu6050", "Gyro: ( %d, %d, %d)", gx, gy, gz);
        ESP_LOGI("mpu6050", "Pitch: %.3f", pitch);
        ESP_LOGI("mpu6050", "Roll: %.3f", roll);
        vTaskDelay(1000/portTICK_RATE_MS);
    }

}

extern "C" void app_main()
{
    xTaskCreatePinnedToCore(&mpu6050_task,"mpu6050_task",2048,NULL,5,NULL,0);
}