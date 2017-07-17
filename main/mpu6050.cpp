#include "mpu6050.hpp"

MPU6050::MPU6050(gpio_num_t scl, gpio_num_t sda, i2c_port_t port) {
    i2c = new I2C(scl, sda, port);
}

MPU6050::~MPU6050() {
    delete(i2c);
}

bool MPU6050::init() {
    if (!i2c -> slave_write(MPU6050_ADDR, PWR_MGMT_1  , 0x00))
        return false;
    if (!i2c -> slave_write(MPU6050_ADDR, SMPLRT_DIV  , 0x07))
        return false;
    if (!i2c -> slave_write(MPU6050_ADDR, CONFIG      , 0x07))
        return false;
    if (!i2c -> slave_write(MPU6050_ADDR, GYRO_CONFIG , 0x18))
        return false;
    if (!i2c -> slave_write(MPU6050_ADDR, ACCEL_CONFIG, 0x01))
        return false;
    return true;
}

short MPU6050::getAccX() {
    uint8_t r[0];
    i2c -> slave_read(MPU6050_ADDR, ACCEL_XOUT_H, r, 2);
    return r[0] << 8 | r[1];
}

short MPU6050::getAccY() {
    uint8_t r[0];
    i2c -> slave_read(MPU6050_ADDR, ACCEL_YOUT_H, r, 2);
    return r[0] << 8 | r[1];
}

short MPU6050::getAccZ() {
    uint8_t r[0];
    i2c -> slave_read(MPU6050_ADDR, ACCEL_ZOUT_H, r, 2);
    return r[0] << 8 | r[1];
}

short MPU6050::getGyroX() {
    uint8_t r[0];
    i2c -> slave_read(MPU6050_ADDR, GYRO_XOUT_H, r, 2);
    return r[0] << 8 | r[1];
}

short MPU6050::getGyroY() {
    uint8_t r[0];
    i2c -> slave_read(MPU6050_ADDR, GYRO_YOUT_H, r, 2);
    return r[0] << 8 | r[1];
}

short MPU6050::getGyroZ() {
    uint8_t r[0];
    i2c -> slave_read(MPU6050_ADDR, GYRO_ZOUT_H, r, 2);
    return r[0] << 8 | r[1];
}

short MPU6050::getTemp() {
    uint8_t r[0];
    i2c -> slave_read(MPU6050_ADDR, TEMP_OUT_H, r, 2);
    return r[0] << 8 | r[1];
}
