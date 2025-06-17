#pragma once

#include <esp_err.h>
#include "esp_event.h"
#include "esp_http_server.h"
#include "web_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

// WiFi配置宏定义
#define ESP32_WIFI_SSID      "ESP32_Robot_Arm"      // WiFi SSID
#define ESP32_WIFI_PASSWORD  "12345678"        // WiFi 密码
#define ESP32_WIFI_CHANNEL   1                 // WiFi 信道
#define ESP32_MAX_STA_CONN   4                 // 最大客户端连接数
#define ESP32_WIFI_MAX_RETRY 5                 // 最大重试次数

// 网页控制器结构体前向声明
struct web_controller;

/**
 * @brief WiFi事件处理函数
 *
 * @param arg 控制器句柄
 * @param event_base 事件基
 * @param event_id 事件ID
 * @param event_data 事件数据
 */
void wifi_event_handler(void* arg, esp_event_base_t event_base,
                      int32_t event_id, void* event_data);

/**
 * @brief 初始化WiFi AP模式
 *
 * @param controller 网页控制器句柄
 * @return esp_err_t 操作结果
 */
esp_err_t init_wifi_ap(struct web_controller *controller);

/**
 * @brief 初始化WiFi STA模式
 *
 * @param handle 网页控制器句柄
 * @param ssid WiFi SSID
 * @param password WiFi 密码
 * @return esp_err_t 操作结果
 */
esp_err_t init_wifi_sta(struct web_controller *handle, const char *ssid, const char *password);

/**
 * @brief 初始化SPIFFS文件系统
 *
 * @return esp_err_t 操作结果
 */
esp_err_t init_www_spiffs(void);

/**
 * @brief 注册Web服务器的所有HTTP处理程序
 *
 * @param server HTTP服务器句柄
 */
void register_wifi_handlers(httpd_handle_t server);

#ifdef __cplusplus
}
#endif 