#include "include/servo_controller.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SERVO";

// 将角度映射为脉冲宽度
static uint16_t map_angle_to_pulse_width(servo_handle_t *handle, uint8_t angle) {
    if (angle < handle->config.min_angle) {
        angle = handle->config.min_angle;
    } else if (angle > handle->config.max_angle) {
        angle = handle->config.max_angle;
    }

    // 应用反转
    if (handle->config.invert) {
        angle = handle->config.max_angle - angle + handle->config.min_angle;
    }

    // 将角度映射到脉冲宽度
    float range = (float)(handle->config.max_angle - handle->config.min_angle);
    float angle_fraction = (float)(angle - handle->config.min_angle) / range;
    
    uint16_t pulse_width = handle->config.min_pulse_width + angle_fraction * 
                          (handle->config.max_pulse_width - handle->config.min_pulse_width);
    
    return pulse_width;
}

// 将脉冲宽度转换为占空比值
static uint16_t pulse_width_to_duty(pca9685_handle_t *pca9685, uint16_t pulse_width_us) {
    // 一个PWM周期的长度 (微秒)
    uint32_t period_us = 1000000 / SERVO_DEFAULT_FREQ;
    
    // 将脉冲宽度转换为占空比值 (0-4095)
    uint16_t duty = (uint16_t)((float)pulse_width_us / (float)period_us * 4096.0f);
    
    if (duty > 4095) {
        duty = 4095;
    }
    
    return duty;
}

esp_err_t servo_init(pca9685_handle_t *pca9685, const servo_config_t *config, servo_handle_t *handle) {
    if (!pca9685 || !config || !handle || config->channel > 15) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 初始化舵机句柄
    handle->pca9685 = pca9685;
    handle->config = *config;
    
    // 设置默认值
    if (handle->config.min_pulse_width == 0) {
        handle->config.min_pulse_width = SERVO_MIN_PULSE_WIDTH_US;
    }
    if (handle->config.max_pulse_width == 0) {
        handle->config.max_pulse_width = SERVO_MAX_PULSE_WIDTH_US;
    }
    if (handle->config.min_angle == 0 && handle->config.max_angle == 0) {
        handle->config.min_angle = SERVO_MIN_ANGLE;
        handle->config.max_angle = SERVO_MAX_ANGLE;
    }
    
    // 确保PCA9685频率设为50Hz (适用于大多数舵机)
    esp_err_t ret = pca9685_set_pwm_frequency(pca9685, SERVO_DEFAULT_FREQ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM frequency to %dHz, err = %d", SERVO_DEFAULT_FREQ, ret);
        return ret;
    }
    
    // 设置舵机到初始位置
    if (handle->config.home_angle > 0 || handle->config.home_angle <= handle->config.max_angle) {
        ret = servo_set_angle(handle, handle->config.home_angle);
    } else {
        // 如果没有指定初始角度，则将舵机设置到中间位置
        uint8_t middle_angle = (handle->config.min_angle + handle->config.max_angle) / 2;
        ret = servo_set_angle(handle, middle_angle);
    }
    
    ESP_LOGI(TAG, "Servo initialized on channel %d", handle->config.channel);
    return ret;
}

esp_err_t servo_set_angle(servo_handle_t *handle, uint8_t angle) {
    if (!handle || angle < handle->config.min_angle || angle > handle->config.max_angle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint16_t pulse_width = map_angle_to_pulse_width(handle, angle);
    return servo_set_pulse_width(handle, pulse_width);
}

esp_err_t servo_set_pulse_width(servo_handle_t *handle, uint16_t pulse_width) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 限制脉冲宽度在有效范围内
    if (pulse_width < handle->config.min_pulse_width) {
        pulse_width = handle->config.min_pulse_width;
    } else if (pulse_width > handle->config.max_pulse_width) {
        pulse_width = handle->config.max_pulse_width;
    }
    
    // 转换为占空比
    uint16_t duty = pulse_width_to_duty(handle->pca9685, pulse_width);
    
    // 设置PWM
    esp_err_t ret = pca9685_set_duty(handle->pca9685, handle->config.channel, duty);
    if (ret == ESP_OK) {
        handle->current_duty = duty;
        
        // 更新当前角度 (近似值)
        float range = (float)(handle->config.max_pulse_width - handle->config.min_pulse_width);
        float pulse_fraction = (float)(pulse_width - handle->config.min_pulse_width) / range;
        handle->current_angle = handle->config.min_angle + pulse_fraction * 
                               (handle->config.max_angle - handle->config.min_angle);
        
        if (handle->config.invert) {
            handle->current_angle = handle->config.max_angle - handle->current_angle + handle->config.min_angle;
        }
    }
    
    return ret;
}

esp_err_t servo_set_home(servo_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (handle->config.home_angle > 0 || handle->config.home_angle <= handle->config.max_angle) {
        return servo_set_angle(handle, handle->config.home_angle);
    } else {
        // 如果没有指定初始角度，则将舵机设置到中间位置
        uint8_t middle_angle = (handle->config.min_angle + handle->config.max_angle) / 2;
        return servo_set_angle(handle, middle_angle);
    }
}

// 平滑移动的任务函数
typedef struct {
    servo_handle_t *handle;
    uint8_t target_angle;
    uint8_t speed;
    TaskHandle_t task_handle;
} smooth_move_params_t;

static void smooth_move_task(void *pvParameters) {
    smooth_move_params_t *params = (smooth_move_params_t *)pvParameters;
    servo_handle_t *handle = params->handle;
    uint8_t target_angle = params->target_angle;
    uint8_t speed = params->speed; // 角度/秒
    
    uint8_t current_angle = handle->current_angle;
    uint8_t step = 1; // 每次移动1度
    
    // 计算延迟时间 (毫秒)
    uint32_t delay_ms = 1000 / speed;
    
    // 移动方向
    int8_t direction = (target_angle > current_angle) ? 1 : -1;
    
    ESP_LOGI(TAG, "Smooth move from %d to %d degrees with speed %d deg/s", 
             current_angle, target_angle, speed);
    
    while (current_angle != target_angle) {
        current_angle += direction * step;
        
        // 确保不会越界
        if ((direction > 0 && current_angle > target_angle) || 
            (direction < 0 && current_angle < target_angle)) {
            current_angle = target_angle;
        }
        
        servo_set_angle(handle, current_angle);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
    
    // 释放资源
    vTaskDelete(NULL);
}

esp_err_t servo_move_smooth(servo_handle_t *handle, uint8_t target_angle, uint8_t speed) {
    if (!handle || target_angle < handle->config.min_angle || 
        target_angle > handle->config.max_angle || speed == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 如果已经在目标角度，不需要移动
    if (handle->current_angle == target_angle) {
        return ESP_OK;
    }
    
    // 分配参数结构体
    smooth_move_params_t *params = malloc(sizeof(smooth_move_params_t));
    if (!params) {
        return ESP_ERR_NO_MEM;
    }
    
    params->handle = handle;
    params->target_angle = target_angle;
    params->speed = speed;
    
    // 创建平滑移动任务
    BaseType_t ret = xTaskCreate(
        smooth_move_task,
        "servo_smooth_move",
        2048,
        params,
        5,
        &params->task_handle
    );
    
    if (ret != pdPASS) {
        free(params);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t servo_stop(servo_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 关闭PWM输出
    return pca9685_set_pwm(handle->pca9685, handle->config.channel, 0, 0);
} 