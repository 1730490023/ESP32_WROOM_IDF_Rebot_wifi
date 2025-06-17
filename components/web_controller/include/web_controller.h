#pragma once

#include <esp_err.h>
#include "robotic_arm.h"
#include "learning_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

// 如果未定义MIN宏，则定义它
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

// 网页控制器结构体定义前向声明
struct web_controller;

/**
 * @brief 网页控制器配置
 */
typedef struct {
    robotic_arm_handle_t *arm;              /*!< 机械臂句柄 */
    learning_handle_t *learning;            /*!< 学习控制器句柄 */
    const char *wifi_ssid;                  /*!< WiFi SSID */
    const char *wifi_password;              /*!< WiFi 密码 */
    bool ap_mode;                           /*!< true: AP模式, false: STA模式 */
    int http_port;                          /*!< HTTP服务器端口，默认80 */
} web_controller_config_t;

/**
 * @brief 网页控制器句柄
 */
typedef struct web_controller* web_controller_handle_t;

/**
 * @brief 初始化网页控制器
 *
 * @param config 网页控制器配置
 * @param handle 返回的网页控制器句柄指针
 * @return esp_err_t
 */
esp_err_t web_controller_init(const web_controller_config_t *config, web_controller_handle_t *handle);

/**
 * @brief 启动网页控制器
 *
 * @param handle 网页控制器句柄
 * @return esp_err_t
 */
esp_err_t web_controller_start(web_controller_handle_t handle);

/**
 * @brief 停止网页控制器
 *
 * @param handle 网页控制器句柄
 * @return esp_err_t
 */
esp_err_t web_controller_stop(web_controller_handle_t handle);

/**
 * @brief 获取IP地址
 *
 * @param handle 网页控制器句柄
 * @return const char* IP地址字符串
 */
const char *web_controller_get_ip(web_controller_handle_t handle);

#ifdef __cplusplus
}
#endif 