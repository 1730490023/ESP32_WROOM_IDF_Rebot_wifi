#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "pca9685.h"

#ifdef __cplusplus
extern "C" {
#endif

// 默认舵机频率 (50Hz适用于大多数舵机)
#define SERVO_DEFAULT_FREQ 50

// 舵机脉冲范围 (微秒)
#define SERVO_MIN_PULSE_WIDTH_US 500  // 0.5ms
#define SERVO_MAX_PULSE_WIDTH_US 2500 // 2.5ms

// 舵机角度范围 (度)
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

/**
 * @brief 舵机配置结构体
 */
typedef struct {
    uint8_t channel;            // PCA9685通道 (0-15)
    uint16_t min_pulse_width;   // 最小脉冲宽度 (微秒)
    uint16_t max_pulse_width;   // 最大脉冲宽度 (微秒)
    uint8_t min_angle;          // 最小角度 (度)
    uint8_t max_angle;          // 最大角度 (度)
    uint8_t home_angle;         // 初始角度 (度)
    bool invert;                // 是否反转方向
} servo_config_t;

/**
 * @brief 舵机句柄结构体
 */
typedef struct {
    pca9685_handle_t *pca9685;  // PCA9685句柄
    servo_config_t config;      // 舵机配置
    uint8_t current_angle;      // 当前角度
    uint16_t current_duty;      // 当前占空比
} servo_handle_t;

/**
 * @brief 初始化舵机
 * 
 * @param pca9685 已初始化的PCA9685句柄
 * @param config 舵机配置
 * @param handle 舵机句柄指针
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t servo_init(pca9685_handle_t *pca9685, const servo_config_t *config, servo_handle_t *handle);

/**
 * @brief 设置舵机角度
 * 
 * @param handle 舵机句柄
 * @param angle 角度 (度)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t servo_set_angle(servo_handle_t *handle, uint8_t angle);

/**
 * @brief 设置舵机脉冲宽度
 * 
 * @param handle 舵机句柄
 * @param pulse_width 脉冲宽度 (微秒)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t servo_set_pulse_width(servo_handle_t *handle, uint16_t pulse_width);

/**
 * @brief 设置舵机到初始位置
 * 
 * @param handle 舵机句柄
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t servo_set_home(servo_handle_t *handle);

/**
 * @brief 平滑地移动舵机到指定角度
 * 
 * @param handle 舵机句柄
 * @param target_angle 目标角度 (度)
 * @param speed 移动速度 (度/秒)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t servo_move_smooth(servo_handle_t *handle, uint8_t target_angle, uint8_t speed);

/**
 * @brief 停止舵机（关闭PWM输出）
 * 
 * @param handle 舵机句柄
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t servo_stop(servo_handle_t *handle);

#ifdef __cplusplus
}
#endif 