#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "freertos/event_groups.h"
#include "cJSON.h"
#include "wifi_handlers.h"
#include "web_controller_internal.h"

static const char *TAG = "wifi_handlers";

// WiFi连接事件组标志位
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// 打印MAC地址
static void print_mac(const char* message, const uint8_t* mac, int aid) {
    ESP_LOGI(TAG, "%s %02x:%02x:%02x:%02x:%02x:%02x, AID=%d",
             message, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], aid);
}

// WiFi事件处理函数
void wifi_event_handler(void* arg, esp_event_base_t event_base,
                        int32_t event_id, void* event_data)
{
    struct web_controller *controller = (struct web_controller *)arg;
    static int retry_num = 0;
    
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_AP_STACONNECTED) {
            wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
            print_mac("Station connected:", event->mac, event->aid);
        } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
            wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
            print_mac("Station disconnected:", event->mac, event->aid);
        } else if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (retry_num < ESP32_WIFI_MAX_RETRY) {
                esp_wifi_connect();
                retry_num++;
                ESP_LOGI(TAG, "重试连接AP");
            } else {
                xEventGroupSetBits(controller->wifi_event_group, WIFI_FAIL_BIT);
            }
            ESP_LOGI(TAG, "连接AP失败");
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "获取IP地址:" IPSTR, IP2STR(&event->ip_info.ip));
            
            // 保存IP地址到控制器
            snprintf(controller->ip_addr, sizeof(controller->ip_addr), IPSTR, 
                     IP2STR(&event->ip_info.ip));
            
            retry_num = 0;
            xEventGroupSetBits(controller->wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

// 初始化WiFi AP模式
esp_err_t init_wifi_ap(struct web_controller *controller)
{
    ESP_LOGI(TAG, "正在初始化WiFi AP模式...");
    
    // 配置AP
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP32_WIFI_SSID,
            .ssid_len = strlen(ESP32_WIFI_SSID),
            .channel = ESP32_WIFI_CHANNEL,
            .password = ESP32_WIFI_PASSWORD,
            .max_connection = ESP32_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    
    if (strlen(ESP32_WIFI_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    // 设置WiFi模式为AP
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // 获取和保存AP IP地址
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (netif) {
        ESP_ERROR_CHECK(esp_netif_get_ip_info(netif, &ip_info));
        snprintf(controller->ip_addr, sizeof(controller->ip_addr), IPSTR, 
                 IP2STR(&ip_info.ip));
    } else {
        // 默认AP IP
        strcpy(controller->ip_addr, "192.168.4.1");
    }
    
    ESP_LOGI(TAG, "WiFi AP初始化完成. SSID:%s 密码:%s 信道:%d IP地址:%s",
             ESP32_WIFI_SSID, ESP32_WIFI_PASSWORD, ESP32_WIFI_CHANNEL, controller->ip_addr);
    
    return ESP_OK;
}

// 初始化WiFi STA模式
esp_err_t init_wifi_sta(struct web_controller *handle, const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "正在初始化WiFi STA模式...");
    
    // 配置STA
    wifi_config_t wifi_config = {
        .sta = {
            /* 设置WiFi SSID */
            .ssid = {0},
            /* 设置WiFi密码 */
            .password = {0},
            /* 此设置用于减少WiFi扫描时间 */
            .scan_method = WIFI_FAST_SCAN,
            /* 启用该标志可禁用在WiFi扫描时发现重复AP */
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            /* 设置要扫描的通道数(1-14) */
            .channel = 0,
            /* 在满足某个阈值后终止扫描 */
            .threshold.rssi = -127,
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };
    
    // 复制SSID和密码
    if (ssid) {
        strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    }
    
    if (password) {
        strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    }
    
    // 设置WiFi模式为STA
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi STA初始化完成.");
    
    /* 等待连接到AP完成或超过最大重试次数 */
    EventBits_t bits = xEventGroupWaitBits(handle->wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          portMAX_DELAY);
    
    /* xEventGroupWaitBits() 返回等待的位, 因此我们可以测试连接成功与否 */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "连接到AP SSID:%s 密码:%s", ssid, password);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "无法连接到AP SSID:%s 密码:%s", ssid, password);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "意外事件");
        return ESP_FAIL;
    }
}

// 初始化SPIFFS文件系统
esp_err_t init_www_spiffs(void)
{
    ESP_LOGI(TAG, "正在挂载 SPIFFS 文件系统...");
    
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/www",
        .partition_label = "www",
        .max_files = 5,
        .format_if_mount_failed = true
    };
    
    // 使用配置挂载SPIFFS文件系统
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "无法挂载或格式化文件系统");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "未找到指定的分区");
        } else {
            ESP_LOGE(TAG, "无法初始化SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info("www", &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "获取SPIFFS分区信息失败");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "分区大小: 总共: %d KB, 已使用: %d KB", total / 1024, used / 1024);
    }
    
    ESP_LOGI(TAG, "SPIFFS 文件系统挂载成功");
    return ESP_OK;
}

// 获取文件类型的HTTP Content-Type
static const char* get_content_type(const char *filename)
{
    if (strstr(filename, ".html")) return "text/html";
    else if (strstr(filename, ".css")) return "text/css";
    else if (strstr(filename, ".js")) return "application/javascript";
    else if (strstr(filename, ".png")) return "image/png";
    else if (strstr(filename, ".jpg")) return "image/jpeg";
    else if (strstr(filename, ".ico")) return "image/x-icon";
    else if (strstr(filename, ".svg")) return "image/svg+xml";
    else if (strstr(filename, ".json")) return "application/json";
    return "text/plain";
}

// 处理HTTP GET请求，提供静态文件
esp_err_t http_get_handler(httpd_req_t *req)
{
    // 获取请求的URI
    const char *uri = req->uri;
    char *filepath = NULL;
    
    // 如果请求根目录，则默认提供index.html
    if (strcmp(uri, "/") == 0) {
        uri = "/index.html";
    }
    
    // 检查URI长度
    size_t uri_len = strlen(uri);
    if (uri_len > 200) {
        ESP_LOGE(TAG, "URI太长: %s", uri);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    
    // 安全地构建文件路径
    size_t base_path_len = 4; // "/www"的长度
    size_t filepath_len = base_path_len + uri_len + 1; // +1 for null terminator
    
    filepath = malloc(filepath_len);
    if (!filepath) {
        ESP_LOGE(TAG, "内存分配失败");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // 构建文件路径
    strcpy(filepath, "/www");
    strcat(filepath, uri);
    
    // 检查文件是否存在
    struct stat file_stat;
    if (stat(filepath, &file_stat) != 0) {
        ESP_LOGE(TAG, "文件不存在: %s", filepath);
        free(filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    
    // 打开文件
    FILE *file = fopen(filepath, "r");
    if (!file) {
        ESP_LOGE(TAG, "无法打开文件: %s", filepath);
        free(filepath);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // 设置Content-Type
    httpd_resp_set_type(req, get_content_type(filepath));
    
    // 发送文件内容
    char *chunk = malloc(1024);
    if (!chunk) {
        fclose(file);
        free(filepath);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    size_t read_bytes;
    do {
        read_bytes = fread(chunk, 1, 1024, file);
        if (read_bytes > 0) {
            httpd_resp_send_chunk(req, chunk, read_bytes);
        }
    } while (read_bytes > 0);
    
    // 发送空块表示结束
    httpd_resp_send_chunk(req, NULL, 0);
    free(chunk);
    fclose(file);
    
    // 日志记录
    const char *success_msg = "文件发送成功";
    ESP_LOGI(TAG, "%s: %s", success_msg, filepath);
    free(filepath);
    return ESP_OK;
}

// 注册HTTP处理程序
httpd_uri_t http_get = {
    .uri       = "/*",
    .method    = HTTP_GET,
    .handler   = http_get_handler,
    .user_ctx  = NULL
};

// 注册Web服务器的所有处理程序
void register_wifi_handlers(httpd_handle_t server)
{
    // 初始化SPIFFS文件系统
    if (init_www_spiffs() != ESP_OK) {
        ESP_LOGE(TAG, "初始化SPIFFS文件系统失败，静态文件服务可能不可用");
    }
    
    httpd_register_uri_handler(server, &http_get);
    ESP_LOGI(TAG, "HTTP处理程序注册成功");
} 