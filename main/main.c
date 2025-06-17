/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "robotic_arm.h"
#include "learning_controller.h"
#include "web_controller.h"

// 添加设置电位器参数的函数声明
esp_err_t set_pot_adjust_params(uint8_t servo_idx, int offset, float scale, bool invert);

static const char *TAG = "MAIN";

// PCA9685连接参数
#define I2C_PORT            I2C_NUM_0
#define I2C_SDA_PIN         GPIO_NUM_22  // 确认SDA引脚为GPIO_NUM_22
#define I2C_SCL_PIN         GPIO_NUM_21  // 确认SCL引脚为GPIO_NUM_21
#define I2C_SPEED           400000    // 降至100KHz
#define PCA9685_ADDR        PCA9685_DEFAULT_ADDR      // PCA9685默认地址
#define SERVO_FREQUENCY     50        // 舵机PWM频率 (50Hz)

// 舵机参数 (根据实际舵机进行调整)
#define BASE_SERVO_CHANNEL     0
#define SHOULDER_SERVO_CHANNEL 1
#define ELBOW_SERVO_CHANNEL    2
#define WRIST_SERVO_CHANNEL    3
#define GRIPPER_SERVO_CHANNEL  4

// 机械臂连杆长度 (单位：毫米，根据实际机械臂调整)
#define ARM_LENGTH      105.0f  // 上臂长度
#define FOREARM_LENGTH  100.0f  // 前臂长度
#define WRIST_LENGTH    50.0f   // 腕部长度
#define BASE_HEIGHT     85.0f   // 基座高度

// 电位器ADC引脚配置
#define POT_BASE_PIN       GPIO_NUM_36  // ADC1_CH0   底座 3号
#define POT_SHOULDER_PIN   GPIO_NUM_39  // ADC1_CH3   下手臂 4号
#define POT_ELBOW_PIN      GPIO_NUM_34  // ADC1_CH6   上手臂 2号
#define POT_WRIST_PIN      GPIO_NUM_35  // ADC1_CH7
#define POT_GRIPPER_PIN    GPIO_NUM_32  // ADC1_CH4
#define MODE_BUTTON_PIN    GPIO_NUM_26  // 保持按钮引脚不变

// 学习控制器NVS命名空间
#define LEARNING_NVS_NAMESPACE "arm_paths"

// 舵机示例演示任务
static void robotic_arm_demo_task(void *pvParameters);

// 学习控制任务
static void learning_task(void *pvParameters);

// Web控制器句柄
static web_controller_handle_t web_controller_handle;

// Web控制任务
static void web_control_task(void *pvParameters);

// 全局变量
static pca9685_handle_t pca9685_handle;
static robotic_arm_handle_t robotic_arm_handle;
static learning_handle_t learning_handle;

// PCA9685基本测试函数
static esp_err_t test_pca9685_basic(i2c_port_t i2c_port, uint8_t device_addr) {
    ESP_LOGI(TAG, "开始PCA9685基本测试，地址: 0x%02X", device_addr);
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // MODE1寄存器
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法写入MODE1寄存器地址: 0x%02X, 错误: %d", device_addr, ret);
        return ret;
    }
    
    uint8_t data;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // MODE1寄存器地址
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法读取MODE1寄存器: 0x%02X, 错误: %d", device_addr, ret);
        return ret;
    }
    
    ESP_LOGI(TAG, "MODE1寄存器值: 0x%02X", data);
    
    // 简单测试 - 写入一个值然后读回
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // MODE1寄存器
    i2c_master_write_byte(cmd, 0x01, true); // 写入值0x01 (ALLCALL位)
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法写入MODE1寄存器值: 0x%02X, 错误: %d", device_addr, ret);
        return ret;
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // MODE1寄存器地址
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法读取MODE1寄存器值: 0x%02X, 错误: %d", device_addr, ret);
        return ret;
    }
    
    ESP_LOGI(TAG, "MODE1寄存器写入后读回值: 0x%02X", data);
    
    if (data == 0x01) {
        ESP_LOGI(TAG, "PCA9685基本测试通过!");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "PCA9685测试失败 - 寄存器值不符合预期");
        return ESP_FAIL;
    }
}

// I2C扫描函数，用于检测总线上的设备
static esp_err_t i2c_scan_devices(i2c_port_t i2c_port) {
    ESP_LOGI(TAG, "开始扫描I2C总线上的设备...");
    uint8_t devices_found = 0;
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "发现I2C设备，地址: 0x%02X", addr);
            devices_found++;
        }
    }
    
    if (devices_found == 0) {
        ESP_LOGW(TAG, "未发现任何I2C设备!");
    } else {
        ESP_LOGI(TAG, "I2C总线扫描完成，共发现 %d 个设备", devices_found);
    }
    
    return (devices_found > 0) ? ESP_OK : ESP_FAIL;
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32机械臂控制系统启动");
    
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 初始化PCA9685
    pca9685_config_t pca9685_config = {
        .i2c_port = I2C_PORT,
        .i2c_addr = PCA9685_ADDR,
        .i2c_speed = I2C_SPEED,
        .sda_pin = I2C_SDA_PIN,
        .scl_pin = I2C_SCL_PIN,
        .external_clock = false,
        .frequency = SERVO_FREQUENCY
    };
    
    // 先配置I2C
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_SPEED,
    };
    
    ret = i2c_param_config(I2C_PORT, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C参数配置失败: %d", ret);
        return;
    }
    
    ret = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C驱动安装失败: %d", ret);
        return;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "I2C驱动已安装，继续执行");
    }
    
    // 延迟一段时间，等待I2C总线稳定
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // 扫描I2C总线上的设备
    i2c_scan_devices(I2C_PORT);
    
    // 先进行PCA9685基本测试
    ESP_LOGI(TAG, "执行PCA9685基本测试");
    esp_err_t test_ret = test_pca9685_basic(I2C_PORT, PCA9685_ADDR);
    if (test_ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685基本测试失败，将不继续初始化机械臂");
        return;
    }
    
    // 继续初始化PCA9685
    ret = pca9685_init(&pca9685_config, &pca9685_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685初始化失败：%u", ret);
        return;
    }
    
    // 配置舵机参数
    servo_config_t servo_configs[ROBOTIC_ARM_MAX_SERVOS] = {
        // 底座舵机
        {
            .channel = BASE_SERVO_CHANNEL,
            .min_pulse_width = 500,    // 0.5ms脉冲宽度
            .max_pulse_width = 2500,   // 2.5ms脉冲宽度
            .min_angle = 0,
            .max_angle = 180,
            .home_angle = 98,
            .invert = false
        },
        // 肩部舵机
        {
            .channel = SHOULDER_SERVO_CHANNEL,
            .min_pulse_width = 500,
            .max_pulse_width = 2500,
            .min_angle = 0,
            .max_angle = 180,
            .home_angle = 60,
            .invert = false
        },
        // 肘部舵机
        {
            .channel = ELBOW_SERVO_CHANNEL,
            .min_pulse_width = 500,
            .max_pulse_width = 2500,
            .min_angle = 0,
            .max_angle = 180,
            .home_angle = 150,
            .invert = false
        },
        // 腕部舵机
        {
            .channel = WRIST_SERVO_CHANNEL,
            .min_pulse_width = 500,
            .max_pulse_width = 2500,
            .min_angle = 0,
            .max_angle = 180,
            .home_angle = 90,
            .invert = false
        },
        // 夹爪舵机
        {
            .channel = GRIPPER_SERVO_CHANNEL,
            .min_pulse_width = 500,
            .max_pulse_width = 2500,
            .min_angle = 0,
            .max_angle = 90,
            .home_angle = 90,
            .invert = false
        }
    };
    
    // 配置机械臂参数
    robotic_arm_config_t robotic_arm_config = {
        .pca9685 = &pca9685_handle,
        .servo_configs = { servo_configs[0], servo_configs[1], servo_configs[2], 
                         servo_configs[3], servo_configs[4] },
        .kinematics = {
            .arm_length = ARM_LENGTH,
            .forearm_length = FOREARM_LENGTH,
            .wrist_length = WRIST_LENGTH,
            .base_height = BASE_HEIGHT
        }
    };
    
    // 初始化机械臂
    ret = robotic_arm_init(&robotic_arm_config, &robotic_arm_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "机械臂初始化失败：%u", ret);
        return;
    }
    
    // 测试舵机控制 - 逐个测试每个舵机
    ESP_LOGI(TAG, "开始测试各个舵机...");
    
    // 测试底座舵机
    ESP_LOGI(TAG, "测试底座舵机...");
    ret = robotic_arm_set_joint_angle(&robotic_arm_handle, ROBOTIC_ARM_BASE_SERVO, 90, true, 30);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "底座舵机控制失败: %d", ret);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // 测试肩部舵机
    ESP_LOGI(TAG, "测试肩部舵机...");
    ret = robotic_arm_set_joint_angle(&robotic_arm_handle, ROBOTIC_ARM_SHOULDER_SERVO, 90, true, 30);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "肩部舵机控制失败: %d", ret);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // 测试肘部舵机
    ESP_LOGI(TAG, "测试肘部舵机...");
    ret = robotic_arm_set_joint_angle(&robotic_arm_handle, ROBOTIC_ARM_ELBOW_SERVO, 90, true, 30);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "肘部舵机控制失败: %d", ret);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // 测试腕部舵机
    ESP_LOGI(TAG, "测试腕部舵机...");
    ret = robotic_arm_set_joint_angle(&robotic_arm_handle, ROBOTIC_ARM_WRIST_SERVO, 90, true, 30);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "腕部舵机控制失败: %d", ret);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // 测试夹爪舵机
    ESP_LOGI(TAG, "测试夹爪舵机...");
    ret = robotic_arm_set_joint_angle(&robotic_arm_handle, ROBOTIC_ARM_GRIPPER_SERVO, 45, true, 30);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "夹爪舵机控制失败: %d", ret);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // 设置回初始位置
    ESP_LOGI(TAG, "舵机测试完成，回到初始位置");
    robotic_arm_set_home(&robotic_arm_handle, true, 30);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // 初始化学习控制器
    learning_pins_config_t learning_pins = {
        .adc_pins = {
            POT_BASE_PIN,
            POT_SHOULDER_PIN,
            POT_ELBOW_PIN,
            POT_WRIST_PIN,
            POT_GRIPPER_PIN
        },
        .mode_button = MODE_BUTTON_PIN
    };
    
    learning_config_t learning_config = {
        .pins = learning_pins,
        .arm = &robotic_arm_handle,
        .nvs_namespace = LEARNING_NVS_NAMESPACE
    };
    
    ret = learning_controller_init(&learning_config, &learning_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "学习控制器初始化失败：%u", ret);
        return;
    }
    
    // 创建学习任务
    xTaskCreate(learning_task, "learning_task", 4096, NULL, 5, NULL);
    
    // 创建示例演示任务
    xTaskCreate(robotic_arm_demo_task, "robotic_arm_demo", 4096, NULL, 5, NULL);
    
    // 创建Web控制任务
    xTaskCreate(web_control_task, "web_control", 8192, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "机械臂初始化完成，开始演示");
}

// 演示任务 - 执行一系列预设动作
static void robotic_arm_demo_task(void *pvParameters)
{
    // 等待所有舵机初始化完成
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // 设置回原点
    ESP_LOGI(TAG, "机械臂回到初始位置");
    robotic_arm_set_home(&robotic_arm_handle, true, 30);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
#if 0
    // 示例1：使用关节角度控制
    ESP_LOGI(TAG, "示例1：使用关节角度控制");
    robotic_arm_angles_t angles1 = {
        .base = 45,
        .shoulder = 120,
        .elbow = 60,
        .wrist = 90,
        .gripper = 30
    };
    robotic_arm_set_angles(&robotic_arm_handle, &angles1, true, 20);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // 示例2：调整夹爪位置
    ESP_LOGI(TAG, "示例2：调整夹爪位置");
    robotic_arm_set_gripper(&robotic_arm_handle, 120, true, 40);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    robotic_arm_set_gripper(&robotic_arm_handle, 60, true, 40);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    
    // 示例3：使用位置控制 (逆运动学)
    ESP_LOGI(TAG, "示例3：使用位置控制 (逆运动学)");
    robotic_arm_position_t pos1 = {
        .x = 200.0f,
        .y = 0.0f,
        .z = 150.0f,
        .roll = 0.0f,
        .pitch = 0.0f,
        .yaw = 90.0f,
        .gripper_angle = 90
    };
    robotic_arm_set_position(&robotic_arm_handle, &pos1, true, 15);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // 示例4：执行简单的抓取动作
    ESP_LOGI(TAG, "示例4：执行简单的抓取动作");
    
    // 移动到目标物体上方
    robotic_arm_position_t pos_grab1 = {
        .x = 180.0f,
        .y = 50.0f,
        .z = 100.0f,
        .roll = 0.0f,
        .pitch = 90.0f,
        .yaw = 0.0f,
        .gripper_angle = 120  // 打开夹爪
    };
    robotic_arm_set_position(&robotic_arm_handle, &pos_grab1, true, 20);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // 下降到抓取位置
    pos_grab1.z = 50.0f;
    robotic_arm_set_position(&robotic_arm_handle, &pos_grab1, true, 15);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    
    // 闭合夹爪抓取物体
    robotic_arm_set_gripper(&robotic_arm_handle, 60, true, 20);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    
    // 提升物体
    pos_grab1.z = 120.0f;
    robotic_arm_set_position(&robotic_arm_handle, &pos_grab1, true, 15);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    
    // 移动到放置位置
    robotic_arm_position_t pos_grab2 = {
        .x = 150.0f,
        .y = -80.0f,
        .z = 120.0f,
        .roll = 0.0f,
        .pitch = 90.0f,
        .yaw = 0.0f,
        .gripper_angle = 60  // 保持夹爪闭合
    };
    robotic_arm_set_position(&robotic_arm_handle, &pos_grab2, true, 20);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // 下降到放置位置
    pos_grab2.z = 50.0f;
    robotic_arm_set_position(&robotic_arm_handle, &pos_grab2, true, 15);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    
    // 打开夹爪释放物体
    robotic_arm_set_gripper(&robotic_arm_handle, 120, true, 20);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    
    // 抬升
    pos_grab2.z = 120.0f;
    robotic_arm_set_position(&robotic_arm_handle, &pos_grab2, true, 15);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // 返回初始位置
    ESP_LOGI(TAG, "演示完成，返回初始位置");
    robotic_arm_set_home(&robotic_arm_handle, true, 15);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
#endif    
    // 结束演示
    ESP_LOGI(TAG, "演示结束");

    // 删除任务
    vTaskDelete(NULL);
}

// 学习控制任务
static void learning_task(void *pvParameters) {
    ESP_LOGI(TAG, "启动学习控制任务");
    
    // 等待初始化完成
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // 设置电位器调整参数 - 根据实际情况调整
    // 参数: 舵机索引, 角度偏移, 缩放系数, 是否反转
    set_pot_adjust_params(ROBOTIC_ARM_BASE_SERVO, 0, 1.0, false);     // 底座
    set_pot_adjust_params(ROBOTIC_ARM_SHOULDER_SERVO, 0, 1.0, false); // 肩部
    set_pot_adjust_params(ROBOTIC_ARM_ELBOW_SERVO, 0, 1.0, true);     // 肘部 (反转)
    set_pot_adjust_params(ROBOTIC_ARM_WRIST_SERVO, 0, 1.0, false);    // 腕部
    set_pot_adjust_params(ROBOTIC_ARM_GRIPPER_SERVO, 0, 1.0, false);  // 夹爪
    
    // 尝试加载演示路径，如果不存在则创建一个简单的演示路径
    esp_err_t ret = learning_load_path(&learning_handle, 0);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "未找到预设路径，创建默认演示路径");
        
        // 创建一个简单的演示路径 - 机械臂做简单动作
        learning_start_recording(&learning_handle);
        
        // 等待一会儿以确保录制开始
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        // 为演示添加几个关键点
        // 原始位置
        robotic_arm_set_home(&robotic_arm_handle, true, 30);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // 点1：前伸
        robotic_arm_angles_t angles1 = {
            .base = 90,
            .shoulder = 45,
            .elbow = 45,
            .wrist = 90,
            .gripper = 90
        };
        robotic_arm_set_angles(&robotic_arm_handle, &angles1, true, 20);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // 点2：侧伸
        robotic_arm_angles_t angles2 = {
            .base = 180,
            .shoulder = 60,
            .elbow = 30,
            .wrist = 90,
            .gripper = 45
        };
        robotic_arm_set_angles(&robotic_arm_handle, &angles2, true, 20);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // 点3：抬高
        robotic_arm_angles_t angles3 = {
            .base = 0,
            .shoulder = 130,
            .elbow = 40,
            .wrist = 90,
            .gripper = 90
        };
        robotic_arm_set_angles(&robotic_arm_handle, &angles3, true, 20);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // 回到原始位置
        robotic_arm_set_home(&robotic_arm_handle, true, 30);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // 停止记录并保存
        learning_stop_recording(&learning_handle);
        
        ESP_LOGI(TAG, "已创建默认演示路径");
    } else {
        ESP_LOGI(TAG, "已找到预设路径，无需创建");
    }
    
    // 设置初始模式
    learning_set_mode(&learning_handle, LEARNING_MODE_MANUAL);
    
    // 主循环
    while (1) {
        // 更新学习控制器
        esp_err_t ret = learning_update(&learning_handle);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "学习控制器更新失败：%d", ret);
        }
        
        // 短暂延时，降低CPU占用
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Web控制任务
static void web_control_task(void *pvParameters)
{
    // 等待所有舵机初始化完成
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "启动Web控制界面");
    
    // 配置Web控制器
    web_controller_config_t web_config = {
        .arm = &robotic_arm_handle,
        .learning = &learning_handle,
        .ap_mode = true,                     // 使用AP模式
        .wifi_ssid = "ESP32_ROBOTIC_ARM",    // AP模式下的SSID
        .wifi_password = "12345678",         // AP模式下的密码
        .http_port = 80                      // HTTP端口
    };
    
    // 初始化Web控制器
    esp_err_t ret = web_controller_init(&web_config, &web_controller_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初始化Web控制器失败：%d", ret);
        vTaskDelete(NULL);
        return;
    }
    
    // 启动Web控制器
    ret = web_controller_start(web_controller_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动Web控制器失败：%d", ret);
        vTaskDelete(NULL);
        return;
    }
    
    // 持续打印IP地址，方便用户连接
    while (1) {
        ESP_LOGI(TAG, "Web控制界面已启动，请连接WiFi: ESP32_ROBOTIC_ARM, 密码: 12345678");
        ESP_LOGI(TAG, "访问地址: http://%s/", web_controller_get_ip(web_controller_handle));
        vTaskDelay(30000 / portTICK_PERIOD_MS); // 每30秒打印一次
    }
    
    // 任务不应该到达此处
    web_controller_stop(web_controller_handle);
    vTaskDelete(NULL);
}

