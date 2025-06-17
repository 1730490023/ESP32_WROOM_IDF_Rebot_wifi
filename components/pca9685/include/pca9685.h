#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// PCA9685默认地址
#define PCA9685_DEFAULT_ADDR 0x40

// PCA9685寄存器地址
#define PCA9685_MODE1        0x00
#define PCA9685_MODE2        0x01
#define PCA9685_SUBADR1      0x02
#define PCA9685_SUBADR2      0x03
#define PCA9685_SUBADR3      0x04
#define PCA9685_ALLCALLADR   0x05
#define PCA9685_LED0_ON_L    0x06
#define PCA9685_LED0_ON_H    0x07
#define PCA9685_LED0_OFF_L   0x08
#define PCA9685_LED0_OFF_H   0x09
#define PCA9685_ALL_LED_ON_L 0xFA
#define PCA9685_ALL_LED_ON_H 0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD
#define PCA9685_PRESCALE     0xFE

// 模式寄存器位
#define PCA9685_MODE1_RESTART 0x80
#define PCA9685_MODE1_EXTCLK  0x40
#define PCA9685_MODE1_AI      0x20 // 自动递增
#define PCA9685_MODE1_SLEEP   0x10
#define PCA9685_MODE1_SUB1    0x08
#define PCA9685_MODE1_SUB2    0x04
#define PCA9685_MODE1_SUB3    0x02
#define PCA9685_MODE1_ALLCALL 0x01

// 模式2寄存器位
#define PCA9685_MODE2_INVRT   0x10
#define PCA9685_MODE2_OCH     0x08 // 输出变化类型
#define PCA9685_MODE2_OUTDRV  0x04 // 输出类型
#define PCA9685_MODE2_OUTNE1  0x02
#define PCA9685_MODE2_OUTNE0  0x01

// 外部时钟频率 (典型值)
#define PCA9685_EXTCLK_FREQ  25000000 // 25 MHz

// 默认内部时钟频率
#define PCA9685_OSC_FREQ     25000000 // 25 MHz

/**
 * @brief PCA9685配置结构体
 */
typedef struct {
    i2c_port_t i2c_port;       // I2C端口
    uint8_t i2c_addr;          // I2C地址
    uint32_t i2c_speed;        // I2C速度
    gpio_num_t sda_pin;        // SDA引脚
    gpio_num_t scl_pin;        // SCL引脚
    bool external_clock;       // 是否使用外部时钟
    uint32_t frequency;        // PWM频率（Hz）
} pca9685_config_t;

/**
 * @brief PCA9685句柄结构体
 */
typedef struct {
    i2c_port_t i2c_port;      // I2C端口
    uint8_t addr;             // I2C地址
    uint32_t osc_freq;        // 振荡器频率
    uint8_t prescale;         // 预分频值
} pca9685_handle_t;

/**
 * @brief 初始化PCA9685
 * 
 * @param config PCA9685配置
 * @param handle PCA9685句柄指针
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t pca9685_init(const pca9685_config_t *config, pca9685_handle_t *handle);

/**
 * @brief 设置PCA9685的PWM频率
 * 
 * @param handle PCA9685句柄
 * @param freq 频率（赫兹）
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t pca9685_set_pwm_frequency(pca9685_handle_t *handle, uint32_t freq);

/**
 * @brief 设置特定通道的PWM
 * 
 * @param handle PCA9685句柄
 * @param channel 通道编号 (0-15)
 * @param on 开启时间 (0-4095)
 * @param off 关闭时间 (0-4095)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t pca9685_set_pwm(pca9685_handle_t *handle, uint8_t channel, uint16_t on, uint16_t off);

/**
 * @brief 设置特定通道的PWM占空比
 * 
 * @param handle PCA9685句柄
 * @param channel 通道编号 (0-15)
 * @param duty 占空比 (0-4095)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t pca9685_set_duty(pca9685_handle_t *handle, uint8_t channel, uint16_t duty);

/**
 * @brief 设置所有通道的PWM
 * 
 * @param handle PCA9685句柄
 * @param on 开启时间 (0-4095)
 * @param off 关闭时间 (0-4095)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t pca9685_set_all_pwm(pca9685_handle_t *handle, uint16_t on, uint16_t off);

/**
 * @brief 设置PCA9685至睡眠状态
 * 
 * @param handle PCA9685句柄
 * @param sleep true睡眠，false唤醒
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t pca9685_set_sleep(pca9685_handle_t *handle, bool sleep);

/**
 * @brief 重启PCA9685
 * 
 * @param handle PCA9685句柄
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t pca9685_restart(pca9685_handle_t *handle);

#ifdef __cplusplus
}
#endif 