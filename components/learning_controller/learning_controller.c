#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "learning_controller.h"

static const char *TAG = "LEARNING";

// NVS键名前缀
#define NVS_PATH_KEY_PREFIX "path_"
#define NVS_PATH_COUNT_KEY "path_count"

// ADC配置常量
#define ADC_ATTEN           ADC_ATTEN_DB_11
#define ADC_BIT_WIDTH       ADC_BITWIDTH_12
#define ADC_MAX_VALUE       4095  // 12位ADC最大值

// 电位器角度调整参数
typedef struct {
    int offset;     // 角度偏移量 (-90到+90)
    float scale;    // 角度缩放系数 (0.5-2.0)
    bool invert;    // 是否反转方向
} pot_adjust_t;

// 默认调整参数
static pot_adjust_t pot_adjusts[ROBOTIC_ARM_MAX_SERVOS] = {
    {0, 1.0, false},  // BASE
    {0, 1.0, false},  // SHOULDER
    {0, 1.0, true},   // ELBOW (默认反转)
    {0, 1.0, false},  // WRIST
    {0, 1.0, false}   // GRIPPER
};

// 按钮防抖时间
#define BUTTON_DEBOUNCE_MS  50

// 静态函数声明
static esp_err_t init_adc_pins(learning_handle_t *handle);
static esp_err_t init_mode_button(learning_handle_t *handle);
static esp_err_t read_potentiometer(learning_handle_t *handle, uint8_t index, uint16_t *value);
static uint8_t map_adc_to_angle(uint16_t adc_value, uint8_t min_angle, uint8_t max_angle, pot_adjust_t *adjust);
static esp_err_t add_path_point(learning_handle_t *handle);
static esp_err_t build_nvs_key(uint8_t path_index, char *key_buffer, size_t buffer_size);
static esp_err_t handle_manual_mode(learning_handle_t *handle);
static esp_err_t handle_recording_mode(learning_handle_t *handle);
static esp_err_t handle_playback_mode(learning_handle_t *handle);
static esp_err_t check_mode_button(learning_handle_t *handle);

// ADC实例
static adc_oneshot_unit_handle_t adc1_handle = NULL;

// 静态变量
static uint32_t last_button_press_time = 0;
static uint32_t last_sample_time = 0;
static bool button_pressed = false;  // 按钮按下状态标志

// ADC通道映射数组
static adc_channel_t adc_channels[ROBOTIC_ARM_MAX_SERVOS] = {0};

// 初始化学习控制器
esp_err_t learning_controller_init(const learning_config_t *config, learning_handle_t *handle) {
    ESP_LOGI(TAG, "初始化学习控制器");
    
    if (config == NULL || handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 复制配置
    memcpy(&handle->config, config, sizeof(learning_config_t));
    
    // 初始化其他字段
    handle->current_mode = LEARNING_MODE_MANUAL;
    handle->current_path_index = 0;
    handle->is_recording = false;
    handle->is_playing = false;
    handle->start_time = 0;
    memset(&handle->current_path, 0, sizeof(robot_path_t));
    
    // 初始化ADC引脚
    esp_err_t ret = init_adc_pins(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初始化ADC引脚失败: %d", ret);
        return ret;
    }
    
    // 初始化模式切换按钮
    ret = init_mode_button(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初始化模式按钮失败: %d", ret);
        return ret;
    }
    
    ESP_LOGI(TAG, "学习控制器初始化完成");
    return ESP_OK;
}

// 初始化ADC引脚
static esp_err_t init_adc_pins(learning_handle_t *handle) {
    ESP_LOGI(TAG, "初始化ADC引脚");
    
    // 配置ADC
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&adc_cfg, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC单元初始化失败: %d", ret);
        return ret;
    }
    
    // 手动映射GPIO到ADC通道
    // 注意: ESP32的GPIO与ADC通道映射是固定的
    for (int i = 0; i < ROBOTIC_ARM_MAX_SERVOS; i++) {
        gpio_num_t pin = handle->config.pins.adc_pins[i];
        adc_channel_t channel;
        
        // 手动映射GPIO到ADC通道
        switch (pin) {
            case GPIO_NUM_36: channel = ADC_CHANNEL_0; break;  // VP
            case GPIO_NUM_37: channel = ADC_CHANNEL_1; break;
            case GPIO_NUM_38: channel = ADC_CHANNEL_2; break;
            case GPIO_NUM_39: channel = ADC_CHANNEL_3; break;  // VN
            case GPIO_NUM_32: channel = ADC_CHANNEL_4; break;
            case GPIO_NUM_33: channel = ADC_CHANNEL_5; break;
            case GPIO_NUM_34: channel = ADC_CHANNEL_6; break;
            case GPIO_NUM_35: channel = ADC_CHANNEL_7; break;
            default:
                ESP_LOGE(TAG, "GPIO %d 不是有效的ADC输入引脚", pin);
                return ESP_ERR_INVALID_ARG;
        }
        
        // 保存通道映射，供后续读取使用
        adc_channels[i] = channel;
        
        // 配置ADC通道
        adc_oneshot_chan_cfg_t chan_cfg = {
            .bitwidth = ADC_BIT_WIDTH,
            .atten = ADC_ATTEN,
        };
        
        ret = adc_oneshot_config_channel(adc1_handle, channel, &chan_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC通道配置失败: %d, 引脚: %d, 通道: %d", ret, pin, channel);
            return ret;
        }
        
        ESP_LOGI(TAG, "已配置ADC引脚 %d -> 通道 %d", pin, channel);
    }
    
    return ESP_OK;
}

// 初始化模式切换按钮
static esp_err_t init_mode_button(learning_handle_t *handle) {
    ESP_LOGI(TAG, "初始化模式切换按钮");
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << handle->config.pins.mode_button),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // 下降沿触发
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO配置失败: %d", ret);
        return ret;
    }
    
    ESP_LOGI(TAG, "已配置模式按钮 GPIO %d", handle->config.pins.mode_button);
    return ESP_OK;
}

// 读取电位器值
static esp_err_t read_potentiometer(learning_handle_t *handle, uint8_t index, uint16_t *value) {
    if (index >= ROBOTIC_ARM_MAX_SERVOS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 检查ADC初始化
    if (adc1_handle == NULL) {
        ESP_LOGE(TAG, "ADC未初始化");
        return ESP_ERR_INVALID_STATE;
    }
    
    adc_channel_t channel = adc_channels[index];
    
    // 读取多次样本并平均
    int sum = 0;
    int valid_samples = 0;
    for (int i = 0; i < LEARNING_ADC_SAMPLES; i++) {
        int sample;
        esp_err_t ret = adc_oneshot_read(adc1_handle, channel, &sample);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC读取失败: %d, 通道: %d", ret, channel);
            continue;
        }
        
        // 有效值检查（避免异常值）
        if (sample < 0 || sample > 4095) {
            ESP_LOGW(TAG, "ADC值异常: %d, 通道: %d", sample, channel);
            continue;
        }
        
        sum += sample;
        valid_samples++;
        vTaskDelay(1 / portTICK_PERIOD_MS); // 短暂延时
    }
    
    // 检查是否有有效样本
    if (valid_samples == 0) {
        ESP_LOGE(TAG, "电位器 %d 没有有效样本", index);
        return ESP_FAIL;
    }
    
    *value = sum / valid_samples;
    return ESP_OK;
}

// 将ADC值映射到角度范围，应用偏移和缩放
static uint8_t map_adc_to_angle(uint16_t adc_value, uint8_t min_angle, uint8_t max_angle, pot_adjust_t *adjust) {
    float angle_f;
    
    // 基础映射 (0-4095 -> min-max)
    if (adjust->invert) {
        // 反转映射
        angle_f = max_angle - ((float)(max_angle - min_angle) * adc_value) / ADC_MAX_VALUE;
    } else {
        // 正常映射
        angle_f = min_angle + ((float)(max_angle - min_angle) * adc_value) / ADC_MAX_VALUE;
    }
    
    // 应用缩放 (围绕中点)
    float mid_angle = (min_angle + max_angle) / 2.0f;
    angle_f = mid_angle + (angle_f - mid_angle) * adjust->scale;
    
    // 应用偏移
    angle_f += adjust->offset;
    
    // 确保角度在有效范围内
    if (angle_f < min_angle) angle_f = min_angle;
    if (angle_f > max_angle) angle_f = max_angle;
    
    return (uint8_t)angle_f;
}

// 添加路径点
static esp_err_t add_path_point(learning_handle_t *handle) {
    if (handle->current_path.point_count >= LEARNING_MAX_PATH_POINTS) {
        ESP_LOGW(TAG, "路径点已达到最大数量");
        return ESP_ERR_NO_MEM;
    }
    
    // 获取当前角度
    robotic_arm_angles_t angles = handle->config.arm->current_angles;
    
    // 检查是否与上一个点相同或非常接近（避免记录重复点位）
    if (handle->current_path.point_count > 0) {
        path_point_t *last_point = &handle->current_path.points[handle->current_path.point_count - 1];
        robotic_arm_angles_t last_angles = last_point->angles;
        
        // 如果所有角度都非常接近上一个点，则跳过记录
        if (abs((int)last_angles.base - (int)angles.base) <= 2 &&
            abs((int)last_angles.shoulder - (int)angles.shoulder) <= 2 &&
            abs((int)last_angles.elbow - (int)angles.elbow) <= 2 &&
            abs((int)last_angles.wrist - (int)angles.wrist) <= 2 &&
            abs((int)last_angles.gripper - (int)angles.gripper) <= 2) {
            
            ESP_LOGD(TAG, "跳过记录重复点位");
            return ESP_OK;
        }
    }
    
    // 计算时间戳
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t timestamp = now - handle->start_time;
    
    // 添加路径点
    path_point_t *point = &handle->current_path.points[handle->current_path.point_count];
    memcpy(&point->angles, &angles, sizeof(robotic_arm_angles_t));
    point->timestamp = timestamp;
    
    handle->current_path.point_count++;
    handle->current_path.total_time_ms = timestamp;
    
    ESP_LOGI(TAG, "添加路径点 #%d, 时间: %lu ms, 角度=[%d,%d,%d,%d,%d]", 
             handle->current_path.point_count, timestamp,
             angles.base, angles.shoulder, angles.elbow, angles.wrist, angles.gripper);
    return ESP_OK;
}

// 构建NVS键名
static esp_err_t build_nvs_key(uint8_t path_index, char *key_buffer, size_t buffer_size) {
    if (path_index >= LEARNING_MAX_PATHS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    snprintf(key_buffer, buffer_size, "%s%u", NVS_PATH_KEY_PREFIX, (uint16_t)path_index);
    return ESP_OK;
}

// 开始路径记录
esp_err_t learning_start_recording(learning_handle_t *handle) {
    ESP_LOGI(TAG, "开始路径记录");
    
    if (handle == NULL || handle->is_recording || handle->is_playing) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 清空当前路径
    memset(&handle->current_path, 0, sizeof(robot_path_t));
    
    // 设置开始时间
    handle->start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    handle->is_recording = true;
    
    // 添加第一个路径点
    add_path_point(handle);
    
    return ESP_OK;
}

// 停止路径记录
esp_err_t learning_stop_recording(learning_handle_t *handle) {
    ESP_LOGI(TAG, "停止路径记录");
    
    if (handle == NULL || !handle->is_recording) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 添加最后一个路径点
    add_path_point(handle);
    
    handle->is_recording = false;
    
    ESP_LOGI(TAG, "路径记录完成，共 %u 个点，总时间 %lu ms", 
             handle->current_path.point_count, handle->current_path.total_time_ms);
    
    // 修改：不再自动保存到路径0，让用户在网页上选择保存
    // 这样用户可以选择保存到哪个路径索引
    
    return ESP_OK;
}

// 开始路径回放
esp_err_t learning_start_playback(learning_handle_t *handle, uint8_t path_index) {
    ESP_LOGI(TAG, "开始路径回放，索引: %d", path_index);
    
    if (handle == NULL || handle->is_recording) {
        ESP_LOGE(TAG, "无法开始回放：句柄为空或已在记录状态");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 如果已经在回放中，先停止当前回放
    if (handle->is_playing) {
        learning_stop_playback(handle);
    }
    
    // 检查路径索引
    if (path_index >= LEARNING_MAX_PATHS) {
        ESP_LOGE(TAG, "路径索引超出范围: %d >= %d", path_index, LEARNING_MAX_PATHS);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 加载路径
    ESP_LOGI(TAG, "正在从NVS加载路径 %d", path_index);
    esp_err_t ret = learning_load_path(handle, path_index);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法加载路径 %d: %d", path_index, ret);
        return ret;
    }
    
    // 检查路径是否为空
    if (handle->current_path.point_count == 0) {
        ESP_LOGW(TAG, "路径为空，无法回放");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "成功加载路径，包含 %d 个点，总时长 %lu ms", 
             handle->current_path.point_count, handle->current_path.total_time_ms);
    
    // 打印路径的第一个和最后一个点的信息
    if (handle->current_path.point_count > 0) {
        path_point_t *first = &handle->current_path.points[0];
        path_point_t *last = &handle->current_path.points[handle->current_path.point_count - 1];
        
        ESP_LOGI(TAG, "第一个点: 时间=%lu ms, 角度=[%d,%d,%d,%d,%d]", 
            first->timestamp, 
            first->angles.base, first->angles.shoulder, 
            first->angles.elbow, first->angles.wrist, first->angles.gripper);
        
        ESP_LOGI(TAG, "最后一个点: 时间=%lu ms, 角度=[%d,%d,%d,%d,%d]", 
            last->timestamp, 
            last->angles.base, last->angles.shoulder, 
            last->angles.elbow, last->angles.wrist, last->angles.gripper);
    }
    
    // 设置开始时间
    handle->start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    handle->is_playing = true;
    handle->current_path_index = path_index;
    
    // 移动到第一个路径点
    path_point_t *first_point = &handle->current_path.points[0];
    ESP_LOGI(TAG, "正在移动到初始位置");
    ret = robotic_arm_set_angles(handle->config.arm, &first_point->angles, true, 30);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法设置初始位置: %d", ret);
        handle->is_playing = false;
        return ret;
    }
    
    ESP_LOGI(TAG, "路径回放开始");
    return ESP_OK;
}

// 停止路径回放
esp_err_t learning_stop_playback(learning_handle_t *handle) {
    ESP_LOGI(TAG, "停止路径回放");
    
    if (handle == NULL || !handle->is_playing) {
        return ESP_ERR_INVALID_STATE;
    }
    
    handle->is_playing = false;
    
    return ESP_OK;
}

// 保存当前路径
esp_err_t learning_save_path(learning_handle_t *handle, uint8_t path_index) {
    ESP_LOGI(TAG, "保存路径到索引 %u", path_index);
    
    if (handle == NULL || path_index >= LEARNING_MAX_PATHS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (handle->current_path.point_count == 0) {
        ESP_LOGW(TAG, "当前路径为空，无法保存");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 打开NVS
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(handle->config.nvs_namespace, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法打开NVS: %d", ret);
        return ret;
    }
    
    // 构建键名
    char key[16];
    build_nvs_key(path_index, key, sizeof(key));
    
    // 保存路径
    ret = nvs_set_blob(nvs_handle, key, &handle->current_path, sizeof(robot_path_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法保存路径: %d", ret);
        nvs_close(nvs_handle);
        return ret;
    }
    
    // 提交更改
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法提交NVS更改: %d", ret);
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "路径成功保存到索引 %u", path_index);
    return ESP_OK;
}

// 加载路径
esp_err_t learning_load_path(learning_handle_t *handle, uint8_t path_index) {
    ESP_LOGI(TAG, "加载路径 %u", path_index);
    
    if (handle == NULL || path_index >= LEARNING_MAX_PATHS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 打开NVS
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(handle->config.nvs_namespace, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法打开NVS: %d", ret);
        return ret;
    }
    
    // 构建键名
    char key[16];
    build_nvs_key(path_index, key, sizeof(key));
    
    // 获取数据大小
    size_t required_size = sizeof(robot_path_t);
    ret = nvs_get_blob(nvs_handle, key, NULL, &required_size);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "无法获取路径大小: %d", ret);
        nvs_close(nvs_handle);
        return ret;
    }
    
    if (ret == ESP_ERR_NVS_NOT_FOUND || required_size != sizeof(robot_path_t)) {
        ESP_LOGW(TAG, "路径 %u 不存在或大小不匹配", path_index);
        nvs_close(nvs_handle);
        return ESP_ERR_NOT_FOUND;
    }
    
    // 加载路径
    ret = nvs_get_blob(nvs_handle, key, &handle->current_path, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法加载路径: %d", ret);
        nvs_close(nvs_handle);
        return ret;
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "成功加载路径 %u, 共 %u 个点, 总时间 %lu ms", 
             path_index, handle->current_path.point_count, handle->current_path.total_time_ms);
    
    return ESP_OK;
}

// 删除路径
esp_err_t learning_delete_path(learning_handle_t *handle, uint8_t path_index) {
    ESP_LOGI(TAG, "删除路径 %u", path_index);
    
    if (handle == NULL || path_index >= LEARNING_MAX_PATHS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 打开NVS
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(handle->config.nvs_namespace, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法打开NVS: %d", ret);
        return ret;
    }
    
    // 构建键名
    char key[16];
    build_nvs_key(path_index, key, sizeof(key));
    
    // 删除键
    ret = nvs_erase_key(nvs_handle, key);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "无法删除路径: %d", ret);
        nvs_close(nvs_handle);
        return ret;
    }
    
    // 提交更改
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法提交NVS更改: %d", ret);
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "路径 %u 已删除", path_index);
    return ESP_OK;
}

// 获取当前模式
learning_mode_t learning_get_mode(learning_handle_t *handle) {
    if (handle == NULL) {
        return LEARNING_MODE_MANUAL;
    }
    
    return handle->current_mode;
}

// 设置当前模式
esp_err_t learning_set_mode(learning_handle_t *handle, learning_mode_t mode) {
    ESP_LOGI(TAG, "设置模式: %d", mode);
    
    if (handle == NULL || mode >= LEARNING_MODE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 处理模式切换
    if (handle->current_mode == mode) {
        return ESP_OK;
    }
    
    // 停止当前模式中的操作
    if (handle->is_recording) {
        learning_stop_recording(handle);
    }
    
    if (handle->is_playing) {
        learning_stop_playback(handle);
    }
    
    // 切换模式
    handle->current_mode = mode;
    
    ESP_LOGI(TAG, "已切换到模式: %d", mode);
    return ESP_OK;
}

// 处理手动模式逻辑
static esp_err_t handle_manual_mode(learning_handle_t *handle) {
    // 初始化angles变量，使用当前机械臂角度作为初始值
    robotic_arm_angles_t angles = handle->config.arm->current_angles;
    static int debug_counter = 0;
    bool values_changed = false;
    
    // 从电位器读取角度
    for (int i = 0; i < ROBOTIC_ARM_MAX_SERVOS; i++) {
        uint16_t adc_value;
        if (read_potentiometer(handle, i, &adc_value) == ESP_OK) {
            // 映射ADC值到舵机角度
            servo_config_t *servo_config = &handle->config.arm->servos[i].config;
            uint8_t angle = map_adc_to_angle(adc_value, servo_config->min_angle, servo_config->max_angle, &pot_adjusts[i]);
            
            // 调试输出
            if (++debug_counter % 100 == 0) {
                ESP_LOGI(TAG, "舵机 %d: ADC=%d, 角度=%d", i, adc_value, angle);
            }
            
            // 检测角度是否有变化
            uint8_t current_angle = 0;
            switch (i) {
                case ROBOTIC_ARM_BASE_SERVO:
                    current_angle = handle->config.arm->current_angles.base;
                    if (abs((int)current_angle - (int)angle) > 2) {
                        values_changed = true;
                    }
                    angles.base = angle;
                    break;
                case ROBOTIC_ARM_SHOULDER_SERVO:
                    current_angle = handle->config.arm->current_angles.shoulder;
                    if (abs((int)current_angle - (int)angle) > 2) {
                        values_changed = true;
                    }
                    angles.shoulder = angle;
                    break;
                case ROBOTIC_ARM_ELBOW_SERVO:
                    current_angle = handle->config.arm->current_angles.elbow;
                    if (abs((int)current_angle - (int)angle) > 2) {
                        values_changed = true;
                    }
                    angles.elbow = angle;
                    break;
                case ROBOTIC_ARM_WRIST_SERVO:
                    current_angle = handle->config.arm->current_angles.wrist;
                    if (abs((int)current_angle - (int)angle) > 2) {
                        values_changed = true;
                    }
                    angles.wrist = angle;
                    break;
                case ROBOTIC_ARM_GRIPPER_SERVO:
                    current_angle = handle->config.arm->current_angles.gripper;
                    if (abs((int)current_angle - (int)angle) > 2) {
                        values_changed = true;
                    }
                    angles.gripper = angle;
                    break;
            }
        } else {
            ESP_LOGW(TAG, "电位器 %d 读取失败", i);
        }
    }
    
    // 只有在角度有变化时才设置舵机角度（避免无效指令）
    if (values_changed) {
        // 设置机械臂角度
        esp_err_t ret = robotic_arm_set_angles(handle->config.arm, &angles, false, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "手动模式：设置舵机角度失败，错误 %d", ret);
            // 尝试逐个设置舵机角度以找出问题
            for (int i = 0; i < ROBOTIC_ARM_MAX_SERVOS; i++) {
                uint8_t angle = 0;
                switch (i) {
                    case ROBOTIC_ARM_BASE_SERVO: angle = angles.base; break;
                    case ROBOTIC_ARM_SHOULDER_SERVO: angle = angles.shoulder; break;
                    case ROBOTIC_ARM_ELBOW_SERVO: angle = angles.elbow; break;
                    case ROBOTIC_ARM_WRIST_SERVO: angle = angles.wrist; break;
                    case ROBOTIC_ARM_GRIPPER_SERVO: angle = angles.gripper; break;
                }
                
                esp_err_t single_ret = robotic_arm_set_joint_angle(handle->config.arm, i, angle, false, 0);
                if (single_ret != ESP_OK) {
                    ESP_LOGE(TAG, "舵机 %d 设置失败，角度 %d，错误 %d", i, angle, single_ret);
                }
            }
        } else {
            ESP_LOGD(TAG, "手动模式：设置角度成功 B:%d S:%d E:%d W:%d G:%d", 
                    angles.base, angles.shoulder, angles.elbow, angles.wrist, angles.gripper);
        }
        return ret;
    }
    
    return ESP_OK;
}

// 处理记录模式逻辑
static esp_err_t handle_recording_mode(learning_handle_t *handle) {
    // 修改录制模式逻辑，不再自动开始录制
    // 只有在已经开始录制状态下才执行录制操作
    if (handle->is_recording) {
        // 先更新手动模式，确保机械臂根据电位器移动
        esp_err_t ret = handle_manual_mode(handle);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "记录模式：更新手动控制失败 %d", ret);
        }
        
        // 检查是否需要添加新的路径点
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_sample_time >= LEARNING_SAMPLE_INTERVAL) {
            last_sample_time = now;
            
            // 确保当前机械臂位置已更新
            if (handle->config.arm->current_angles.base == 0 &&
                handle->config.arm->current_angles.shoulder == 0 &&
                handle->config.arm->current_angles.elbow == 0 &&
                handle->config.arm->current_angles.wrist == 0 &&
                handle->config.arm->current_angles.gripper == 0) {
                ESP_LOGW(TAG, "记录模式：当前机械臂角度全为零，可能未正确初始化");
                return ESP_OK;
            }
            
            return add_path_point(handle);
        }
    } else {
        // 如果没有录制，则仍然使用手动模式控制机械臂
        // 这样用户可以调整机械臂位置，然后点击网页上的"开始录制"按钮
        return handle_manual_mode(handle);
    }
    
    return ESP_OK;
}

// 处理回放模式逻辑
static esp_err_t handle_playback_mode(learning_handle_t *handle) {
    // 修改回放模式逻辑，不再自动开始回放
    // 只有在已经开始回放状态下才执行回放操作
    if (!handle->is_playing) {
        // 如果没有回放，则使用手动模式控制机械臂
        // 这样用户可以调整机械臂位置，然后点击网页上的"开始回放"按钮
        return handle_manual_mode(handle);
    }
    
    // 检查路径点数量是否正常
    if (handle->current_path.point_count <= 1) {
        ESP_LOGW(TAG, "回放模式：路径点数量不足，无法回放");
        learning_stop_playback(handle);
        return ESP_OK;
    }
    
    // 输出一次所有点的信息用于调试
    static bool debug_points_printed = false;
    if (!debug_points_printed) {
        ESP_LOGI(TAG, "路径点信息（共%d个点）：", handle->current_path.point_count);
        for (int i = 0; i < handle->current_path.point_count; i++) {
            path_point_t *point = &handle->current_path.points[i];
            ESP_LOGI(TAG, "点#%d: 时间=%lu ms, 角度=[%d,%d,%d,%d,%d]", i,
                    point->timestamp,
                    point->angles.base, point->angles.shoulder,
                    point->angles.elbow, point->angles.wrist, point->angles.gripper);
        }
        debug_points_printed = true;
    }
    
    // 计算当前时间
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now - handle->start_time;
    
    // 检查是否播放完成
    if (elapsed >= handle->current_path.total_time_ms) {
        ESP_LOGI(TAG, "路径回放完成");
        handle->is_playing = false;
        debug_points_printed = false;  // 重置调试标志，以便下次回放时重新打印
        return ESP_OK;
    }
    
    // 找到当前应该播放的点
    int current_index = 0;
    int next_index = 1;  // 至少有2个点
    float interpolation_factor = 0.0f;
    
    // 查找当前和下一个路径点
    bool found_valid_segment = false;
    for (int i = 0; i < handle->current_path.point_count - 1; i++) {
        if (elapsed >= handle->current_path.points[i].timestamp && 
            elapsed < handle->current_path.points[i + 1].timestamp) {
            current_index = i;
            next_index = i + 1;
            found_valid_segment = true;
            
            // 计算插值因子 (0.0-1.0)
            uint32_t segment_duration = handle->current_path.points[next_index].timestamp - 
                                        handle->current_path.points[current_index].timestamp;
            uint32_t segment_elapsed = elapsed - handle->current_path.points[current_index].timestamp;
            
            if (segment_duration > 0) {
                interpolation_factor = (float)segment_elapsed / (float)segment_duration;
                // 确保插值因子在有效范围内
                if (interpolation_factor < 0.0f) interpolation_factor = 0.0f;
                if (interpolation_factor > 1.0f) interpolation_factor = 1.0f;
            } else {
                interpolation_factor = 0.0f; // 防止除以零
            }
            
            ESP_LOGD(TAG, "回放：当前点 #%d, 下一点 #%d, 插值因子 %.2f, 已过时间 %lu ms", 
                    current_index, next_index, interpolation_factor, elapsed);
            break;
        }
    }
    
    // 如果没找到有效的插值段，则使用最后的点
    if (!found_valid_segment) {
        if (elapsed < handle->current_path.points[0].timestamp) {
            // 当前时间早于第一个点的时间戳
            current_index = 0;
            next_index = 0;
            interpolation_factor = 0.0f;
        } else {
            // 当前时间晚于最后一个点的时间戳
            current_index = handle->current_path.point_count - 1;
            next_index = handle->current_path.point_count - 1;
            interpolation_factor = 0.0f;
        }
        ESP_LOGD(TAG, "回放：未找到有效的插值段，使用点 #%d, 已过时间 %lu ms", current_index, elapsed);
    }
    
    // 获取当前和下一个点的角度
    path_point_t *current_point = &handle->current_path.points[current_index];
    path_point_t *next_point = &handle->current_path.points[next_index];
    
    // 每10次更新输出一次调试信息
    static int debug_counter = 0;
    if (++debug_counter % 100 == 0) {
        ESP_LOGI(TAG, "回放：当前点 #%d B:%d S:%d E:%d W:%d G:%d -> 下一点 #%d B:%d S:%d E:%d W:%d G:%d, 因子:%.2f", 
                current_index,
                current_point->angles.base, current_point->angles.shoulder, 
                current_point->angles.elbow, current_point->angles.wrist, 
                current_point->angles.gripper,
                next_index,
                next_point->angles.base, next_point->angles.shoulder, 
                next_point->angles.elbow, next_point->angles.wrist, 
                next_point->angles.gripper,
                interpolation_factor);
    }
    
    // 线性插值计算当前角度
    robotic_arm_angles_t angles;
    
    // 确保我们针对有效的插值段才进行插值
    if (current_index != next_index) {
        angles.base = current_point->angles.base + 
                    (int)((float)(next_point->angles.base - current_point->angles.base) * interpolation_factor);
        angles.shoulder = current_point->angles.shoulder + 
                        (int)((float)(next_point->angles.shoulder - current_point->angles.shoulder) * interpolation_factor);
        angles.elbow = current_point->angles.elbow + 
                    (int)((float)(next_point->angles.elbow - current_point->angles.elbow) * interpolation_factor);
        angles.wrist = current_point->angles.wrist + 
                    (int)((float)(next_point->angles.wrist - current_point->angles.wrist) * interpolation_factor);
        angles.gripper = current_point->angles.gripper + 
                        (int)((float)(next_point->angles.gripper - current_point->angles.gripper) * interpolation_factor);
    } else {
        // 使用当前点的角度
        angles = current_point->angles;
    }
    
    // 检查与机械臂当前角度是否有明显不同
    robotic_arm_angles_t current_angles = handle->config.arm->current_angles;
    bool angles_changed = false;
    
    if (abs((int)current_angles.base - (int)angles.base) > 2 ||
        abs((int)current_angles.shoulder - (int)angles.shoulder) > 2 ||
        abs((int)current_angles.elbow - (int)angles.elbow) > 2 ||
        abs((int)current_angles.wrist - (int)angles.wrist) > 2 ||
        abs((int)current_angles.gripper - (int)angles.gripper) > 2) {
        angles_changed = true;
    }
    
    // 只有在角度有变化时才发送命令
    if (angles_changed) {
        // 设置机械臂角度
        esp_err_t ret = robotic_arm_set_angles(handle->config.arm, &angles, false, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "回放：设置角度失败，错误 %d", ret);
            // 尝试单独设置每个舵机，找出哪个出问题
            for (int i = 0; i < ROBOTIC_ARM_MAX_SERVOS; i++) {
                uint8_t angle = 0;
                switch (i) {
                    case ROBOTIC_ARM_BASE_SERVO: angle = angles.base; break;
                    case ROBOTIC_ARM_SHOULDER_SERVO: angle = angles.shoulder; break;
                    case ROBOTIC_ARM_ELBOW_SERVO: angle = angles.elbow; break;
                    case ROBOTIC_ARM_WRIST_SERVO: angle = angles.wrist; break;
                    case ROBOTIC_ARM_GRIPPER_SERVO: angle = angles.gripper; break;
                }
                
                esp_err_t single_ret = robotic_arm_set_joint_angle(handle->config.arm, i, angle, false, 0);
                if (single_ret != ESP_OK) {
                    ESP_LOGE(TAG, "舵机 %d 设置失败，角度 %d，错误 %d", i, angle, single_ret);
                }
            }
        } else {
            if (debug_counter % 100 == 0) {
                ESP_LOGI(TAG, "回放：插值角度 B:%d S:%d E:%d W:%d G:%d 设置成功", 
                        angles.base, angles.shoulder, angles.elbow, angles.wrist, angles.gripper);
            }
        }
    }
    
    return ESP_OK;
}

// 检查模式按钮状态
static esp_err_t check_mode_button(learning_handle_t *handle) {
    // 读取按钮状态
    int button_state = gpio_get_level(handle->config.pins.mode_button);
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // 按钮状态机处理
    if (!button_pressed) {
        // 当前按钮未被按下，检测新的按下事件
        if (button_state == 0) { // 按钮被按下 (低电平)
            // 防抖
            if (now - last_button_press_time > BUTTON_DEBOUNCE_MS) {
                ESP_LOGI(TAG, "检测到按钮按下");
                button_pressed = true;
                last_button_press_time = now;
            }
        }
    } else {
        // 当前按钮已被按下，等待松开
        if (button_state == 1) { // 按钮松开 (高电平)
            // 防抖
            if (now - last_button_press_time > BUTTON_DEBOUNCE_MS) {
                ESP_LOGI(TAG, "检测到按钮松开，切换模式");
                button_pressed = false;
                
                // 切换到下一个模式
                learning_mode_t next_mode = (handle->current_mode + 1) % LEARNING_MODE_MAX;
                return learning_set_mode(handle, next_mode);
            }
        }
    }
    
    return ESP_OK;
}

// 执行控制器更新
esp_err_t learning_update(learning_handle_t *handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 不再使用按键切换模式，而是完全依靠Web界面控制
    // 注释掉按键检测代码
    /*
    // 检查模式按钮
    esp_err_t ret = check_mode_button(handle);
    if (ret != ESP_OK) {
        return ret;
    }
    */
    
    esp_err_t ret = ESP_OK;
    
    // 根据当前模式执行不同的逻辑
    switch (handle->current_mode) {
        case LEARNING_MODE_MANUAL:
            ret = handle_manual_mode(handle);
            break;
            
        case LEARNING_MODE_RECORD:
            ret = handle_recording_mode(handle);
            break;
            
        case LEARNING_MODE_PLAYBACK:
            ret = handle_playback_mode(handle);
            break;
            
        default:
            ret = ESP_ERR_INVALID_STATE;
            break;
    }
    
    return ret;
}

// 设置电位器角度映射调整参数
esp_err_t set_pot_adjust_params(uint8_t servo_idx, int offset, float scale, bool invert) {
    if (servo_idx >= ROBOTIC_ARM_MAX_SERVOS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 范围检查
    if (offset < -90) offset = -90;
    if (offset > 90) offset = 90;
    
    if (scale < 0.5f) scale = 0.5f;
    if (scale > 2.0f) scale = 2.0f;
    
    // 设置参数
    pot_adjusts[servo_idx].offset = offset;
    pot_adjusts[servo_idx].scale = scale;
    pot_adjusts[servo_idx].invert = invert;
    
    ESP_LOGI(TAG, "设置舵机 %d 参数: 偏移=%d, 缩放=%.2f, 反转=%d", 
             servo_idx, offset, scale, invert ? 1 : 0);
    
    return ESP_OK;
}