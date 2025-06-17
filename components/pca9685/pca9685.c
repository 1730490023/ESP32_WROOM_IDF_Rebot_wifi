#include "pca9685.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <inttypes.h>

static const char *TAG = "PCA9685";

// 读取寄存器
static esp_err_t pca9685_read_reg(pca9685_handle_t *handle, uint8_t reg, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(handle->i2c_port, cmd, 5000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02x, err = %d", reg, ret);
    }
    return ret;
}

// 写入寄存器
static esp_err_t pca9685_write_reg(pca9685_handle_t *handle, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(handle->i2c_port, cmd, 5000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02x with data 0x%02x, err = %d", reg, data, ret);
    }
    return ret;
}

esp_err_t pca9685_init(const pca9685_config_t *config, pca9685_handle_t *handle) {
    if (!config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 配置I2C
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->sda_pin,
        .scl_io_num = config->scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = config->i2c_speed,
    };
    
    // 注意：不再安装I2C驱动，假设驱动已经在主程序中安装
    // 只进行参数配置
    esp_err_t ret = i2c_param_config(config->i2c_port, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters, err = %d", ret);
        return ret;
    }
    
    ESP_LOGI(TAG, "使用已安装的I2C驱动");
    
    // 增加启动延迟，让PCA9685有足够时间完成启动和内部初始化
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // 初始化句柄
    handle->i2c_port = config->i2c_port;
    handle->addr = config->i2c_addr;
    handle->osc_freq = config->external_clock ? PCA9685_EXTCLK_FREQ : PCA9685_OSC_FREQ;
    
    // 软件重置
    ret = pca9685_restart(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart PCA9685, err = %d", ret);
        return ret;
    }
    
    // 配置MODE2
    ret = pca9685_write_reg(handle, PCA9685_MODE2, PCA9685_MODE2_OUTDRV);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 配置MODE1 (自动递增)
    ret = pca9685_write_reg(handle, PCA9685_MODE1, PCA9685_MODE1_AI);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 设置PWM频率
    if (config->frequency > 0) {
        ret = pca9685_set_pwm_frequency(handle, config->frequency);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    
    ESP_LOGI(TAG, "PCA9685 initialized successfully at address 0x%02X", handle->addr);
    return ESP_OK;
}

esp_err_t pca9685_set_pwm_frequency(pca9685_handle_t *handle, uint32_t freq) {
    if (!handle || freq == 0 || freq > 1526) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 计算预分频值
    float prescaleval = ((float)handle->osc_freq / (float)(4096 * freq)) - 1;
    if (prescaleval < 3) {
        prescaleval = 3;
    } else if (prescaleval > 255) {
        prescaleval = 255;
    }
    
    uint8_t prescale = (uint8_t)prescaleval;
    handle->prescale = prescale;
    
    // 读取当前MODE1
    uint8_t mode1;
    esp_err_t ret = pca9685_read_reg(handle, PCA9685_MODE1, &mode1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 进入睡眠模式
    uint8_t sleep_mode1 = (mode1 & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
    ret = pca9685_write_reg(handle, PCA9685_MODE1, sleep_mode1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 设置预分频值
    ret = pca9685_write_reg(handle, PCA9685_PRESCALE, prescale);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 恢复原MODE1（唤醒）
    ret = pca9685_write_reg(handle, PCA9685_MODE1, mode1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 等待振荡器启动
    vTaskDelay(1 / portTICK_PERIOD_MS);
    
    // 设置restart位
    ret = pca9685_write_reg(handle, PCA9685_MODE1, mode1 | PCA9685_MODE1_RESTART);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // ESP_LOGI(TAG, "PCA9685 frequency set to %ld  Hz (prescale: %ld)", freq, prescale);
    return ESP_OK;
}

esp_err_t pca9685_set_pwm(pca9685_handle_t *handle, uint8_t channel, uint16_t on, uint16_t off) {
    if (!handle || channel > 15 || on > 4095 || off > 4095) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t reg_base = PCA9685_LED0_ON_L + (channel * 4);
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_base, true);                  // 起始寄存器
    i2c_master_write_byte(cmd, on & 0xFF, true);                 // LED_ON_L
    i2c_master_write_byte(cmd, (on >> 8) & 0x0F, true);          // LED_ON_H
    i2c_master_write_byte(cmd, off & 0xFF, true);                // LED_OFF_L
    i2c_master_write_byte(cmd, (off >> 8) & 0x0F, true);         // LED_OFF_H
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(handle->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM for channel %d, err = %d", channel, ret);
    }
    
    return ret;
}

esp_err_t pca9685_set_duty(pca9685_handle_t *handle, uint8_t channel, uint16_t duty) {
    if (!handle || channel > 15 || duty > 4095) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return pca9685_set_pwm(handle, channel, 0, duty);
}

esp_err_t pca9685_set_all_pwm(pca9685_handle_t *handle, uint16_t on, uint16_t off) {
    if (!handle || on > 4095 || off > 4095) {
        return ESP_ERR_INVALID_ARG;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, PCA9685_ALL_LED_ON_L, true);      // 起始寄存器
    i2c_master_write_byte(cmd, on & 0xFF, true);                 // ALL_LED_ON_L
    i2c_master_write_byte(cmd, (on >> 8) & 0x0F, true);          // ALL_LED_ON_H
    i2c_master_write_byte(cmd, off & 0xFF, true);                // ALL_LED_OFF_L
    i2c_master_write_byte(cmd, (off >> 8) & 0x0F, true);         // ALL_LED_OFF_H
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(handle->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set all PWM, err = %d", ret);
    }
    
    return ret;
}

esp_err_t pca9685_set_sleep(pca9685_handle_t *handle, bool sleep) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t mode1;
    esp_err_t ret = pca9685_read_reg(handle, PCA9685_MODE1, &mode1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (sleep) {
        // 进入睡眠模式
        uint8_t sleep_mode1 = (mode1 & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
        ret = pca9685_write_reg(handle, PCA9685_MODE1, sleep_mode1);
    } else {
        // 唤醒
        uint8_t wake_mode1 = mode1 & ~PCA9685_MODE1_SLEEP;
        ret = pca9685_write_reg(handle, PCA9685_MODE1, wake_mode1);
        
        if (ret == ESP_OK) {
            // 如果之前处于restart模式，则设置restart位
            if (mode1 & PCA9685_MODE1_RESTART) {
                vTaskDelay(1 / portTICK_PERIOD_MS);
                ret = pca9685_write_reg(handle, PCA9685_MODE1, wake_mode1 | PCA9685_MODE1_RESTART);
            }
        }
    }
    
    return ret;
}

esp_err_t pca9685_restart(pca9685_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Restarting PCA9685 at address 0x%02X", handle->addr);
    
    // 尝试先读取一次当前值，测试通信
    uint8_t mode1_val = 0;
    esp_err_t ret = pca9685_read_reg(handle, PCA9685_MODE1, &mode1_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MODE1 register before restart, err = %d", ret);
        // 尝试继续，不返回错误
    } else {
        ESP_LOGI(TAG, "Current MODE1 value: 0x%02X", mode1_val);
    }
    
    // 等待一段时间
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    // 将MODE1寄存器设为0
    ret = pca9685_write_reg(handle, PCA9685_MODE1, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write MODE1 register during restart, err = %d", ret);
        return ret;
    }
    
    // 增加延迟时间
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    // 再次读取确认
    ret = pca9685_read_reg(handle, PCA9685_MODE1, &mode1_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MODE1 register after restart, err = %d", ret);
        // 尝试继续，不返回错误
    } else {
        ESP_LOGI(TAG, "MODE1 value after restart: 0x%02X", mode1_val);
    }
    
    return ESP_OK;
} 