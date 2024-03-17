#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include <inttypes.h>

#define I2C_MASTER_SCL_IO 5        // 定义 I2C 主机的时钟引脚
#define I2C_MASTER_SDA_IO 4        // 定义 I2C 主机的数据引脚
#define I2C_MASTER_NUM I2C_NUM_1    // 定义 I2C 主机号
#define MPU6050_REG_PWR_MGMT_1 0x6B // 电源管理寄存器地址
#define MPU6050_REG_ACCEL_CONFIG 0x1C // 加速度计配置寄存器地址
#define MPU6050_REG_GYRO_CONFIG 0x1B // 陀螺仪配置寄存器地址
#define MPU6050_ADDR 0x68           // MPU6050 的 I2C 地址
#define MPU6050_REG_WHO_AM_I 0x75   // Who Am I 寄存器地址

void i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void mpu6050_init() {
    // 初始化 I2C 总线
    i2c_master_init();

    // 配置 MPU6050 寄存器
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_REG_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0x00, true); // 设置电源管理寄存器，唤醒 MPU6050
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(100 / portTICK_PERIOD_MS); // 延时等待

    // 配置加速度计和陀螺仪
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_REG_ACCEL_CONFIG, true);
    i2c_master_write_byte(cmd, 0x00, true); // 设置加速度计配置寄存器
    i2c_master_write_byte(cmd, MPU6050_REG_GYRO_CONFIG, true);
    i2c_master_write_byte(cmd, 0x00, true); // 设置陀螺仪配置寄存器
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000);
    i2c_cmd_link_delete(cmd);
}


void mpu6050_read_data() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    uint8_t data[14];
    i2c_master_read(cmd, data, 13, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[13], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000);
    
   int16_t AcX = (data[0] << 8) | data[1];   // X轴加速度
   int16_t AcY = (data[2] << 8) | data[3];   // Y轴加速度
   int16_t AcZ = (data[4] << 8) | data[5];   // Z轴加速度
   int16_t Temp = (data[6] << 8) | data[7];  // 温度
   int16_t GyX = (data[8] << 8) | data[9];   // X轴陀螺仪
   int16_t GyY = (data[10] << 8) | data[11]; // Y轴陀螺仪
   int16_t GyZ = (data[12] << 8) | data[13]; // Z轴陀螺仪
    printf("AcX: %d, AcY: %d, AcZ: %d\n", AcX, AcY, AcZ);
    printf("Temperature: %d\n", Temp);
    printf("GyX: %d, GyY: %d, GyZ: %d\n", GyX, GyY, GyZ);
    printf("\n");
    i2c_cmd_link_delete(cmd);  // 清除命令句柄
}

uint8_t mpu6050_read_who_am_i() {
    // 初始化 I2C 总线
    i2c_master_init();

    // 读取 Who Am I 寄存器
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_REG_WHO_AM_I, true);
    i2c_master_start(cmd); // 重新开始一个新的传输
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000);
    i2c_cmd_link_delete(cmd);

    return data;
}

void app_main() {
    i2c_master_init();
    mpu6050_init();
    uint8_t who_am_i = mpu6050_read_who_am_i(); // 读取 Who Am I 寄存器
    printf("MPU6050 Who Am I: 0x%02X\n", who_am_i);
    while(1) {
        mpu6050_read_data();
        vTaskDelay(1000); // 每秒读取一次数据
    }
}