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
    uint8_t h = 0, l = 0;
    h = i2c -> slave_read_byte(MPU6050_ADDR, ACCEL_XOUT_H);
    l = i2c -> slave_read_byte(MPU6050_ADDR, ACCEL_XOUT_L);
    return h << 8 | l;
}

short MPU6050::getAccY() {
    uint8_t h = 0, l = 0;
    h = i2c -> slave_read_byte(MPU6050_ADDR, ACCEL_YOUT_H);
    l = i2c -> slave_read_byte(MPU6050_ADDR, ACCEL_YOUT_L);
    return h << 8 | l;
}

short MPU6050::getAccZ() {
    uint8_t h = 0, l = 0;
    h = i2c -> slave_read_byte(MPU6050_ADDR, ACCEL_ZOUT_H);
    l = i2c -> slave_read_byte(MPU6050_ADDR, ACCEL_ZOUT_L);
    return h << 8 | l;
}

short MPU6050::getGyroX() {
    uint8_t h = 0, l = 0;
    h = i2c -> slave_read_byte(MPU6050_ADDR, GYRO_XOUT_H);
    l = i2c -> slave_read_byte(MPU6050_ADDR, GYRO_XOUT_L);
    return h << 8 | l;
}

short MPU6050::getGyroY() {
    uint8_t h = 0, l = 0;
    h = i2c -> slave_read_byte(MPU6050_ADDR, GYRO_YOUT_H);
    l = i2c -> slave_read_byte(MPU6050_ADDR, GYRO_YOUT_L);
    return h << 8 | l;
}

short MPU6050::getGyroZ() {
    uint8_t h = 0, l = 0;
    h = i2c -> slave_read_byte(MPU6050_ADDR, GYRO_ZOUT_H);
    l = i2c -> slave_read_byte(MPU6050_ADDR, GYRO_ZOUT_L);
    return h << 8 | l;
}

short MPU6050::getTemp() {
    uint8_t h = 0, l = 0;
    h = i2c -> slave_read_byte(MPU6050_ADDR, TEMP_OUT_H);
    l = i2c -> slave_read_byte(MPU6050_ADDR, TEMP_OUT_L);
    return h << 8 | l;
}
