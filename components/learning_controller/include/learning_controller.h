#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/adc.h"
#include "robotic_arm.h"
#include "nvs_flash.h"
#include "nvs.h"

#ifdef __cplusplus
extern "C" {
#endif

// 学习控制器常量定义
#define LEARNING_MAX_PATH_POINTS 100    // 最大记录的路径点数量
#define LEARNING_MAX_PATHS       5      // 最大保存的路径数量
#define LEARNING_ADC_SAMPLES     10     // ADC采样次数（取平均值）
#define LEARNING_SAMPLE_INTERVAL 100    // 采样间隔时间（毫秒）

// 学习模式枚举
typedef enum {
    LEARNING_MODE_MANUAL = 0,   // 手动控制模式（使用电位器）
    LEARNING_MODE_RECORD,       // 路径记录模式
    LEARNING_MODE_PLAYBACK,     // 路径回放模式
    LEARNING_MODE_MAX
} learning_mode_t;

// 路径点结构体
typedef struct {
    robotic_arm_angles_t angles; // 机械臂关节角度
    uint32_t timestamp;          // 时间戳（相对于路径开始）
} path_point_t;

// 路径结构体
typedef struct {
    path_point_t points[LEARNING_MAX_PATH_POINTS]; // 路径点数组
    uint16_t point_count;                          // 路径点数量
    uint32_t total_time_ms;                        // 路径总时间（毫秒）
} robot_path_t;

// 学习控制器引脚配置
typedef struct {
    gpio_num_t adc_pins[ROBOTIC_ARM_MAX_SERVOS];  // 电位器ADC引脚
    gpio_num_t mode_button;                       // 模式切换按钮引脚
} learning_pins_config_t;

// 学习控制器配置
typedef struct {
    learning_pins_config_t pins;     // 引脚配置
    robotic_arm_handle_t *arm;       // 机械臂句柄
    char *nvs_namespace;             // NVS命名空间
} learning_config_t;

// 学习控制器句柄
typedef struct {
    learning_config_t config;        // 控制器配置
    learning_mode_t current_mode;    // 当前学习模式
    uint8_t current_path_index;      // 当前路径索引
    robot_path_t current_path;       // 当前路径
    bool is_recording;               // 是否正在记录
    bool is_playing;                 // 是否正在回放
    uint32_t start_time;             // 记录/回放开始时间
} learning_handle_t;

/**
 * @brief 初始化学习控制器
 * 
 * @param config 控制器配置
 * @param handle 控制器句柄指针
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t learning_controller_init(const learning_config_t *config, learning_handle_t *handle);

/**
 * @brief 开始路径记录
 * 
 * @param handle 控制器句柄
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t learning_start_recording(learning_handle_t *handle);

/**
 * @brief 停止路径记录
 * 
 * @param handle 控制器句柄
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t learning_stop_recording(learning_handle_t *handle);

/**
 * @brief 开始路径回放
 * 
 * @param handle 控制器句柄
 * @param path_index 路径索引
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t learning_start_playback(learning_handle_t *handle, uint8_t path_index);

/**
 * @brief 停止路径回放
 * 
 * @param handle 控制器句柄
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t learning_stop_playback(learning_handle_t *handle);

/**
 * @brief 保存当前路径到特定索引
 * 
 * @param handle 控制器句柄
 * @param path_index 路径索引
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t learning_save_path(learning_handle_t *handle, uint8_t path_index);

/**
 * @brief 加载特定索引的路径
 * 
 * @param handle 控制器句柄
 * @param path_index 路径索引
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t learning_load_path(learning_handle_t *handle, uint8_t path_index);

/**
 * @brief 删除特定索引的路径
 * 
 * @param handle 控制器句柄
 * @param path_index 路径索引
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t learning_delete_path(learning_handle_t *handle, uint8_t path_index);

/**
 * @brief 获取当前模式
 * 
 * @param handle 控制器句柄
 * @return learning_mode_t 当前模式
 */
learning_mode_t learning_get_mode(learning_handle_t *handle);

/**
 * @brief 设置当前模式
 * 
 * @param handle 控制器句柄
 * @param mode 模式
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t learning_set_mode(learning_handle_t *handle, learning_mode_t mode);

/**
 * @brief 执行一次学习控制器更新（在任务中周期调用）
 * 
 * @param handle 控制器句柄
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t learning_update(learning_handle_t *handle);

/**
 * @brief 设置电位器角度映射调整参数
 * 
 * @param servo_idx 舵机索引 (0-4)
 * @param offset 角度偏移量 (-90到+90)
 * @param scale 角度缩放系数 (0.5-2.0)
 * @param invert 是否反转方向
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t set_pot_adjust_params(uint8_t servo_idx, int offset, float scale, bool invert);

#ifdef __cplusplus
}
#endif 