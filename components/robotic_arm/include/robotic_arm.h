#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "servo_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

// 机械臂舵机数量
#define ROBOTIC_ARM_MAX_SERVOS 5

// 机械臂舵机索引
#define ROBOTIC_ARM_BASE_SERVO     0  // 底座舵机（旋转）
#define ROBOTIC_ARM_SHOULDER_SERVO 1  // 肩部舵机
#define ROBOTIC_ARM_ELBOW_SERVO    2  // 肘部舵机
#define ROBOTIC_ARM_WRIST_SERVO    3  // 腕部舵机
#define ROBOTIC_ARM_GRIPPER_SERVO  4  // 夹爪舵机

// 机械臂位姿结构体
typedef struct {
    float x;              // X坐标 (mm)
    float y;              // Y坐标 (mm)
    float z;              // Z坐标 (mm)
    float roll;           // 滚转角度 (度)
    float pitch;          // 俯仰角度 (度)
    float yaw;            // 偏航角度 (度)
    uint8_t gripper_angle; // 夹爪角度 (度)
} robotic_arm_position_t;

// 机械臂关节角度结构体
typedef struct {
    uint8_t base;         // 底座角度 (度)
    uint8_t shoulder;     // 肩部角度 (度)
    uint8_t elbow;        // 肘部角度 (度)
    uint8_t wrist;        // 腕部角度 (度)
    uint8_t gripper;      // 夹爪角度 (度)
} robotic_arm_angles_t;

// 机械臂连杆参数结构体
typedef struct {
    float arm_length;     // 上臂长度 (mm)
    float forearm_length; // 前臂长度 (mm)
    float wrist_length;   // 腕部长度 (mm)
    float base_height;    // 基座高度 (mm)
} robotic_arm_kinematics_t;

// 机械臂配置结构体
typedef struct {
    pca9685_handle_t *pca9685;      // PCA9685句柄
    servo_config_t servo_configs[ROBOTIC_ARM_MAX_SERVOS]; // 舵机配置数组
    robotic_arm_kinematics_t kinematics;  // 机械臂运动学参数
} robotic_arm_config_t;

// 机械臂句柄结构体
typedef struct {
    servo_handle_t servos[ROBOTIC_ARM_MAX_SERVOS]; // 舵机句柄数组
    robotic_arm_kinematics_t kinematics;           // 机械臂运动学参数
    robotic_arm_angles_t current_angles;           // 当前关节角度
    robotic_arm_position_t current_position;       // 当前末端位姿
} robotic_arm_handle_t;

/**
 * @brief 初始化机械臂
 * 
 * @param config 机械臂配置
 * @param handle 机械臂句柄指针
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t robotic_arm_init(const robotic_arm_config_t *config, robotic_arm_handle_t *handle);

/**
 * @brief 设置机械臂关节角度
 * 
 * @param handle 机械臂句柄
 * @param angles 角度结构体
 * @param smooth 是否平滑移动
 * @param speed 平滑移动时的速度 (度/秒)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t robotic_arm_set_angles(robotic_arm_handle_t *handle, 
                                const robotic_arm_angles_t *angles, 
                                bool smooth, uint8_t speed);

/**
 * @brief 设置机械臂特定关节的角度
 * 
 * @param handle 机械臂句柄
 * @param servo_idx 舵机索引 (0-4)
 * @param angle 角度 (度)
 * @param smooth 是否平滑移动
 * @param speed 平滑移动时的速度 (度/秒)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t robotic_arm_set_joint_angle(robotic_arm_handle_t *handle, 
                                    uint8_t servo_idx, uint8_t angle, 
                                    bool smooth, uint8_t speed);

/**
 * @brief 设置机械臂末端位置 (逆运动学)
 * 
 * @param handle 机械臂句柄
 * @param position 位置结构体
 * @param smooth 是否平滑移动
 * @param speed 平滑移动时的速度 (度/秒)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t robotic_arm_set_position(robotic_arm_handle_t *handle, 
                                 const robotic_arm_position_t *position, 
                                 bool smooth, uint8_t speed);

/**
 * @brief 获取机械臂末端位置 (正运动学)
 * 
 * @param handle 机械臂句柄
 * @param position 位置结构体指针，用于存储计算结果
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t robotic_arm_get_position(robotic_arm_handle_t *handle, 
                                 robotic_arm_position_t *position);

/**
 * @brief 设置夹爪开合
 * 
 * @param handle 机械臂句柄
 * @param angle 角度 (0:完全闭合, 90:中间位置, 180:完全打开)
 * @param smooth 是否平滑移动
 * @param speed 平滑移动时的速度 (度/秒)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t robotic_arm_set_gripper(robotic_arm_handle_t *handle, 
                                uint8_t angle, 
                                bool smooth, uint8_t speed);

/**
 * @brief 设置机械臂到初始位置
 * 
 * @param handle 机械臂句柄
 * @param smooth 是否平滑移动
 * @param speed 平滑移动时的速度 (度/秒)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t robotic_arm_set_home(robotic_arm_handle_t *handle, 
                             bool smooth, uint8_t speed);

/**
 * @brief 机械臂停止（关闭所有舵机PWM输出）
 * 
 * @param handle 机械臂句柄
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t robotic_arm_stop(robotic_arm_handle_t *handle);

#ifdef __cplusplus
}
#endif 