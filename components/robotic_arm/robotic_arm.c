#include "include/robotic_arm.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ROBOTIC_ARM";

// 角度转弧度
static inline float deg_to_rad(float deg) {
    return deg * M_PI / 180.0f;
}

// 弧度转角度
static inline float rad_to_deg(float rad) {
    return rad * 180.0f / M_PI;
}

// 正运动学计算 (从关节角度计算末端位置)
static esp_err_t forward_kinematics(robotic_arm_handle_t *handle, robotic_arm_position_t *position) {
    if (!handle || !position) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 获取当前关节角度并转换为弧度
    float base_rad = deg_to_rad(handle->current_angles.base);
    float shoulder_rad = deg_to_rad(handle->current_angles.shoulder);
    float elbow_rad = deg_to_rad(handle->current_angles.elbow);
    float wrist_rad = deg_to_rad(handle->current_angles.wrist);
    
    // 获取机械臂参数
    float arm_length = handle->kinematics.arm_length;
    float forearm_length = handle->kinematics.forearm_length;
    float wrist_length = handle->kinematics.wrist_length;
    float base_height = handle->kinematics.base_height;
    
    // 计算位置
    float r_shoulder = arm_length * sin(shoulder_rad);
    float z_shoulder = arm_length * cos(shoulder_rad);
    
    float r_elbow = forearm_length * sin(shoulder_rad + elbow_rad);
    float z_elbow = forearm_length * cos(shoulder_rad + elbow_rad);
    
    float r_wrist = wrist_length * sin(shoulder_rad + elbow_rad + wrist_rad);
    float z_wrist = wrist_length * cos(shoulder_rad + elbow_rad + wrist_rad);
    
    float r = r_shoulder + r_elbow + r_wrist;
    float z = base_height + z_shoulder + z_elbow + z_wrist;
    
    // 设置位置
    position->x = r * cos(base_rad);
    position->y = r * sin(base_rad);
    position->z = z;
    
    // 计算末端姿态
    position->roll = 0; // 简化模型，假设没有滚转
    position->pitch = rad_to_deg(shoulder_rad + elbow_rad + wrist_rad);
    position->yaw = handle->current_angles.base;
    position->gripper_angle = handle->current_angles.gripper;
    
    return ESP_OK;
}

// 逆运动学计算 (从末端位置计算关节角度)
static esp_err_t inverse_kinematics(robotic_arm_handle_t *handle, 
                                   const robotic_arm_position_t *position, 
                                   robotic_arm_angles_t *angles) {
    if (!handle || !position || !angles) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 获取机械臂参数
    float arm_length = handle->kinematics.arm_length;
    float forearm_length = handle->kinematics.forearm_length;
    float wrist_length = handle->kinematics.wrist_length;
    float base_height = handle->kinematics.base_height;
    
    // 计算在XY平面上的投影长度
    float r = sqrtf(position->x * position->x + position->y * position->y);
    
    // 计算基座旋转角度
    float base_angle = atan2f(position->y, position->x);
    
    // 调整Z坐标 (减去基座高度)
    float z = position->z - base_height;
    
    // 计算末端与肩部的水平和垂直距离
    float wrist_pitch_rad = deg_to_rad(position->pitch);
    float wx = r - wrist_length * cosf(wrist_pitch_rad);
    float wz = z - wrist_length * sinf(wrist_pitch_rad);
    
    // 计算肩部到腕部的距离
    float sw = sqrtf(wx * wx + wz * wz);
    
    // 检查目标位置是否在工作范围内
    if (sw > arm_length + forearm_length) {
        ESP_LOGE(TAG, "Position out of reach: %.2f > %.2f", 
                 sw, arm_length + forearm_length);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 使用余弦定理计算肘部角度
    float elbow_rad = acosf((arm_length * arm_length + forearm_length * forearm_length - sw * sw) / 
                            (2 * arm_length * forearm_length));
    
    // 计算肩部角度
    float shoulder_rad = atan2f(wz, wx) + 
                        acosf((arm_length * arm_length + sw * sw - forearm_length * forearm_length) / 
                             (2 * arm_length * sw));
    
    // 计算腕部角度 (使末端方向符合要求)
    float wrist_rad = wrist_pitch_rad - shoulder_rad - elbow_rad;
    
    // 转换为角度
    angles->base = rad_to_deg(base_angle);
    angles->shoulder = rad_to_deg(shoulder_rad);
    angles->elbow = rad_to_deg(elbow_rad);
    angles->wrist = rad_to_deg(wrist_rad);
    angles->gripper = position->gripper_angle;
    
    // 限制角度范围 (根据实际舵机限制)
    if (angles->base < 0) angles->base += 180;
    if (angles->base > 180) angles->base = 180;
    
    if (angles->shoulder < 0) angles->shoulder = 0;
    if (angles->shoulder > 180) angles->shoulder = 180;
    
    if (angles->elbow < 0) angles->elbow = 0;
    if (angles->elbow > 180) angles->elbow = 180;
    
    if (angles->wrist < 0) angles->wrist = 0;
    if (angles->wrist > 180) angles->wrist = 180;
    
    if (angles->gripper < 0) angles->gripper = 0;
    if (angles->gripper > 180) angles->gripper = 180;
    
    return ESP_OK;
}

esp_err_t robotic_arm_init(const robotic_arm_config_t *config, robotic_arm_handle_t *handle) {
    if (!config || !handle || !config->pca9685) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 初始化机械臂参数
    handle->kinematics = config->kinematics;
    
    // 初始化舵机
    for (int i = 0; i < ROBOTIC_ARM_MAX_SERVOS; i++) {
        esp_err_t ret = servo_init(config->pca9685, &config->servo_configs[i], &handle->servos[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize servo %d, err = %u", i, ret);
            return ret;
        }
    }
    
    // 初始化当前角度
    handle->current_angles.base = handle->servos[ROBOTIC_ARM_BASE_SERVO].current_angle;
    handle->current_angles.shoulder = handle->servos[ROBOTIC_ARM_SHOULDER_SERVO].current_angle;
    handle->current_angles.elbow = handle->servos[ROBOTIC_ARM_ELBOW_SERVO].current_angle;
    handle->current_angles.wrist = handle->servos[ROBOTIC_ARM_WRIST_SERVO].current_angle;
    handle->current_angles.gripper = handle->servos[ROBOTIC_ARM_GRIPPER_SERVO].current_angle;
    
    // 计算当前位置
    forward_kinematics(handle, &handle->current_position);
    
    ESP_LOGI(TAG, "Robotic arm initialized with %d servos", ROBOTIC_ARM_MAX_SERVOS);
    ESP_LOGI(TAG, "Current angles: base=%d, shoulder=%d, elbow=%d, wrist=%d, gripper=%d", 
             handle->current_angles.base, handle->current_angles.shoulder,
             handle->current_angles.elbow, handle->current_angles.wrist,
             handle->current_angles.gripper);
    
    return ESP_OK;
}

esp_err_t robotic_arm_set_joint_angle(robotic_arm_handle_t *handle, 
                                    uint8_t servo_idx, uint8_t angle, 
                                    bool smooth, uint8_t speed) {
    if (!handle || servo_idx >= ROBOTIC_ARM_MAX_SERVOS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    if (smooth && speed > 0) {
        ret = servo_move_smooth(&handle->servos[servo_idx], angle, speed);
    } else {
        ret = servo_set_angle(&handle->servos[servo_idx], angle);
    }
    
    if (ret == ESP_OK) {
        // 更新当前角度
        switch (servo_idx) {
            case ROBOTIC_ARM_BASE_SERVO:
                handle->current_angles.base = angle;
                break;
            case ROBOTIC_ARM_SHOULDER_SERVO:
                handle->current_angles.shoulder = angle;
                break;
            case ROBOTIC_ARM_ELBOW_SERVO:
                handle->current_angles.elbow = angle;
                break;
            case ROBOTIC_ARM_WRIST_SERVO:
                handle->current_angles.wrist = angle;
                break;
            case ROBOTIC_ARM_GRIPPER_SERVO:
                handle->current_angles.gripper = angle;
                break;
        }
        
        // 更新当前位置
        forward_kinematics(handle, &handle->current_position);
    }
    
    return ret;
}

esp_err_t robotic_arm_set_angles(robotic_arm_handle_t *handle, 
                               const robotic_arm_angles_t *angles, 
                               bool smooth, uint8_t speed) {
    if (!handle || !angles) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    
    // 设置各个关节角度
    ret = robotic_arm_set_joint_angle(handle, ROBOTIC_ARM_BASE_SERVO, angles->base, smooth, speed);
    if (ret != ESP_OK) return ret;
    
    ret = robotic_arm_set_joint_angle(handle, ROBOTIC_ARM_SHOULDER_SERVO, angles->shoulder, smooth, speed);
    if (ret != ESP_OK) return ret;
    
    ret = robotic_arm_set_joint_angle(handle, ROBOTIC_ARM_ELBOW_SERVO, angles->elbow, smooth, speed);
    if (ret != ESP_OK) return ret;
    
    ret = robotic_arm_set_joint_angle(handle, ROBOTIC_ARM_WRIST_SERVO, angles->wrist, smooth, speed);
    if (ret != ESP_OK) return ret;
    
    ret = robotic_arm_set_joint_angle(handle, ROBOTIC_ARM_GRIPPER_SERVO, angles->gripper, smooth, speed);
    
    return ret;
}

esp_err_t robotic_arm_set_position(robotic_arm_handle_t *handle, 
                                 const robotic_arm_position_t *position, 
                                 bool smooth, uint8_t speed) {
    if (!handle || !position) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 计算逆运动学
    robotic_arm_angles_t angles;
    esp_err_t ret = inverse_kinematics(handle, position, &angles);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Inverse kinematics calculation failed, err = %u", ret);
        return ret;
    }
    
    // 设置关节角度
    ret = robotic_arm_set_angles(handle, &angles, smooth, speed);
    if (ret == ESP_OK) {
        // 更新当前位置
        handle->current_position = *position;
    }
    
    return ret;
}

esp_err_t robotic_arm_get_position(robotic_arm_handle_t *handle, 
                                 robotic_arm_position_t *position) {
    if (!handle || !position) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 使用正运动学计算当前位置
    esp_err_t ret = forward_kinematics(handle, position);
    
    return ret;
}

esp_err_t robotic_arm_set_gripper(robotic_arm_handle_t *handle, 
                                uint8_t angle, 
                                bool smooth, uint8_t speed) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 设置夹爪角度
    esp_err_t ret = robotic_arm_set_joint_angle(handle, ROBOTIC_ARM_GRIPPER_SERVO, 
                                              angle, smooth, speed);
    
    return ret;
}

esp_err_t robotic_arm_set_home(robotic_arm_handle_t *handle, 
                             bool smooth, uint8_t speed) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 定义初始位置 (所有舵机回到中间位置)
    robotic_arm_angles_t home_angles = {
        .base = 97,       // 基座中间位置
        .shoulder = 90,   // 肩部抬起
        .elbow = 90,      // 肘部伸直
        .wrist = 90,      // 腕部水平
        .gripper = 90     // 夹爪半开
    };
    
    // 设置到初始位置
    esp_err_t ret = robotic_arm_set_angles(handle, &home_angles, smooth, speed);
    
    return ret;
}

esp_err_t robotic_arm_stop(robotic_arm_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = ESP_OK;
    
    // 停止所有舵机
    for (int i = 0; i < ROBOTIC_ARM_MAX_SERVOS; i++) {
        esp_err_t temp_ret = servo_stop(&handle->servos[i]);
        if (temp_ret != ESP_OK) {
            ret = temp_ret;
        }
    }
    
    return ret;
} 