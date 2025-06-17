#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_http_server.h"
#include "robotic_arm.h"
#include "learning_controller.h"

// Web控制器完整结构体定义，供内部使用
struct web_controller {
    robotic_arm_handle_t *arm;          // 机械臂句柄
    learning_handle_t *learning;        // 学习控制器句柄
    EventGroupHandle_t wifi_event_group; // WiFi事件组
    bool ap_mode;                       // WiFi模式
    char ip_addr[16];                   // IP地址
    httpd_handle_t server;              // HTTP服务器句柄
    char *html_content;                 // HTML页面内容
    SemaphoreHandle_t json_mutex;       // JSON操作互斥锁
}; 