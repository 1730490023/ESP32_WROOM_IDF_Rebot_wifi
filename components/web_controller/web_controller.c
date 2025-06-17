#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "cJSON.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "web_controller.h"
#include "wifi_handlers.h"
#include "web_controller_internal.h"

// 定义MIN宏(如果未定义)
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

static const char *TAG = "WEB_CONTROLLER";

// WiFi连接事件组标志位
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// 默认HTTP端口
#define DEFAULT_HTTP_PORT 80

// 内置HTML页面大小
#define MAX_HTML_SIZE 15000  // 15KB

// 函数声明
static esp_err_t init_http_server(struct web_controller *handle);
static char *generate_html_content(void);
static esp_err_t init_spiffs(void);

// 外部WiFi函数声明
extern void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
extern esp_err_t init_wifi_ap(struct web_controller *controller);
extern esp_err_t init_wifi_sta(struct web_controller *handle, const char *ssid, const char *password);

// HTTP请求处理函数
static esp_err_t index_handler(httpd_req_t *req)
{
    struct web_controller *controller = req->user_ctx;
    
    // 从SPIFFS读取HTML文件
    FILE* f = fopen("/www/index.html", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "无法打开index.html文件，使用备用HTML内容");
        // 使用备用的HTML内容
        httpd_resp_set_type(req, "text/html");
        httpd_resp_sendstr(req, controller->html_content);
        return ESP_OK;
    }
    
    // 获取文件大小
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    // 分配内存并读取文件
    char *buffer = malloc(fsize + 1);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "内存分配失败");
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    size_t read_size = fread(buffer, 1, fsize, f);
    fclose(f);
    
    if (read_size != fsize) {
        ESP_LOGE(TAG, "读取文件失败");
        free(buffer);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    buffer[fsize] = '\0';
    
    // 发送HTML内容
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, buffer, fsize);
    
    free(buffer);
    return ESP_OK;
}

// 获取机械臂状态API
static esp_err_t arm_status_handler(httpd_req_t *req)
{
    struct web_controller *controller = req->user_ctx;
    cJSON *response = cJSON_CreateObject();
    
    xSemaphoreTake(controller->json_mutex, portMAX_DELAY);
    
    // 获取当前角度 - 使用控制器中存储的当前角度
    robotic_arm_angles_t angles = controller->arm->current_angles;
    
    cJSON_AddNumberToObject(response, "base", angles.base);
    cJSON_AddNumberToObject(response, "shoulder", angles.shoulder);
    cJSON_AddNumberToObject(response, "elbow", angles.elbow);
    cJSON_AddNumberToObject(response, "wrist", angles.wrist);
    cJSON_AddNumberToObject(response, "gripper", angles.gripper);
    
    // 获取学习控制器模式
    learning_mode_t mode = learning_get_mode(controller->learning);
    cJSON_AddNumberToObject(response, "mode", (int)mode);
    
    // 添加录制和回放状态标志
    cJSON_AddBoolToObject(response, "is_recording", controller->learning->is_recording);
    cJSON_AddBoolToObject(response, "is_playing", controller->learning->is_playing);
    
    // 序列化JSON并发送
    char *json_str = cJSON_PrintUnformatted(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json_str);
    
    free(json_str);
    cJSON_Delete(response);
    xSemaphoreGive(controller->json_mutex);
    
    return ESP_OK;
}

// 控制机械臂关节API
static esp_err_t arm_control_handler(httpd_req_t *req)
{
    struct web_controller *controller = req->user_ctx;
    char content[100];
    size_t recv_size = MIN(req->content_len, sizeof(content) - 1);
    
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    xSemaphoreTake(controller->json_mutex, portMAX_DELAY);
    
    cJSON *root = cJSON_Parse(content);
    cJSON *response = cJSON_CreateObject();
    bool success = false;
    
    if (root != NULL) {
        robotic_arm_angles_t angles;
        bool has_angles = false;
        int joint_idx = -1;
        float angle = 0;
        
        // 检查是单关节控制还是多关节控制
        cJSON *joint = cJSON_GetObjectItem(root, "joint");
        cJSON *angle_item = cJSON_GetObjectItem(root, "angle");
        
        if (joint && angle_item && cJSON_IsNumber(joint) && cJSON_IsNumber(angle_item)) {
            // 单关节控制
            joint_idx = joint->valueint;
            angle = angle_item->valuedouble;
            
            esp_err_t ret = ESP_OK;
            if (joint_idx >= 0 && joint_idx < ROBOTIC_ARM_MAX_SERVOS) {
                ret = robotic_arm_set_joint_angle(controller->arm, joint_idx, angle, true, 30);
                success = (ret == ESP_OK);
            }
        } else {
            // 多关节控制
            cJSON *base = cJSON_GetObjectItem(root, "base");
            cJSON *shoulder = cJSON_GetObjectItem(root, "shoulder");
            cJSON *elbow = cJSON_GetObjectItem(root, "elbow");
            cJSON *wrist = cJSON_GetObjectItem(root, "wrist");
            cJSON *gripper = cJSON_GetObjectItem(root, "gripper");
            
            // 获取当前角度
            angles = controller->arm->current_angles;
            
            if (base && cJSON_IsNumber(base)) {
                angles.base = base->valuedouble;
                has_angles = true;
            }
            
            if (shoulder && cJSON_IsNumber(shoulder)) {
                angles.shoulder = shoulder->valuedouble;
                has_angles = true;
            }
            
            if (elbow && cJSON_IsNumber(elbow)) {
                angles.elbow = elbow->valuedouble;
                has_angles = true;
            }
            
            if (wrist && cJSON_IsNumber(wrist)) {
                angles.wrist = wrist->valuedouble;
                has_angles = true;
            }
            
            if (gripper && cJSON_IsNumber(gripper)) {
                angles.gripper = gripper->valuedouble;
                has_angles = true;
            }
            
            if (has_angles) {
                esp_err_t ret = robotic_arm_set_angles(controller->arm, &angles, true, 30);
                success = (ret == ESP_OK);
            }
        }
        
        cJSON_Delete(root);
    }
    
    cJSON_AddBoolToObject(response, "success", success);
    char *json_str = cJSON_PrintUnformatted(response);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json_str);
    
    free(json_str);
    cJSON_Delete(response);
    xSemaphoreGive(controller->json_mutex);
    
    return ESP_OK;
}

// 学习控制相关API
static esp_err_t learning_control_handler(httpd_req_t *req)
{
    struct web_controller *controller = req->user_ctx;
    char content[100];
    size_t recv_size = MIN(req->content_len, sizeof(content) - 1);
    
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    xSemaphoreTake(controller->json_mutex, portMAX_DELAY);
    
    cJSON *root = cJSON_Parse(content);
    cJSON *response = cJSON_CreateObject();
    bool success = false;
    char message[64] = "操作失败";
    
    if (root != NULL) {
        cJSON *action = cJSON_GetObjectItem(root, "action");
        cJSON *path_idx_item = cJSON_GetObjectItem(root, "path_index");
        
        int path_idx = 0;
        if (path_idx_item && cJSON_IsNumber(path_idx_item)) {
            path_idx = path_idx_item->valueint;
        }
        
        if (action && cJSON_IsString(action)) {
            const char *action_str = action->valuestring;
            
            if (strcmp(action_str, "set_mode") == 0) {
                cJSON *mode_item = cJSON_GetObjectItem(root, "mode");
                if (mode_item && cJSON_IsNumber(mode_item)) {
                    int mode = mode_item->valueint;
                    if (mode >= 0 && mode < LEARNING_MODE_MAX) {
                        esp_err_t err = learning_set_mode(controller->learning, (learning_mode_t)mode);
                        if (err == ESP_OK) {
                            success = true;
                            snprintf(message, sizeof(message), "设置模式成功: %d", mode);
                        }
                    }
                }
            } else if (strcmp(action_str, "start_recording") == 0) {
                esp_err_t err = learning_start_recording(controller->learning);
                if (err == ESP_OK) {
                    success = true;
                    strcpy(message, "开始记录");
                }
            } else if (strcmp(action_str, "stop_recording") == 0) {
                esp_err_t err = learning_stop_recording(controller->learning);
                if (err == ESP_OK) {
                    success = true;
                    strcpy(message, "停止记录");
                }
            } else if (strcmp(action_str, "save_path") == 0) {
                ESP_LOGI(TAG, "处理save_path请求，路径索引: %d", path_idx);
                
                // 检查当前路径是否有点
                if (controller->learning->current_path.point_count == 0) {
                    ESP_LOGW(TAG, "当前路径为空，无法保存");
                    strcpy(message, "当前路径为空，无法保存");
                } else {
                    esp_err_t err = learning_save_path(controller->learning, path_idx);
                    if (err == ESP_OK) {
                        success = true;
                        snprintf(message, sizeof(message), "保存路径 %d 成功，共 %u 个点", 
                                path_idx, controller->learning->current_path.point_count);
                        ESP_LOGI(TAG, "%s", message);
                    } else {
                        snprintf(message, sizeof(message), "保存路径 %d 失败，错误: %d", path_idx, err);
                        ESP_LOGE(TAG, "%s", message);
                    }
                }
            } else if (strcmp(action_str, "load_path") == 0) {
                esp_err_t err = learning_load_path(controller->learning, path_idx);
                if (err == ESP_OK) {
                    success = true;
                    snprintf(message, sizeof(message), "加载路径 %d 成功", path_idx);
                }
            } else if (strcmp(action_str, "delete_path") == 0) {
                esp_err_t err = learning_delete_path(controller->learning, path_idx);
                if (err == ESP_OK) {
                    success = true;
                    snprintf(message, sizeof(message), "删除路径 %d 成功", path_idx);
                }
            } else if (strcmp(action_str, "start_playback") == 0) {
                esp_err_t err = learning_start_playback(controller->learning, path_idx);
                if (err == ESP_OK) {
                    success = true;
                    snprintf(message, sizeof(message), "开始回放路径 %d", path_idx);
                }
            } else if (strcmp(action_str, "stop_playback") == 0) {
                esp_err_t err = learning_stop_playback(controller->learning);
                if (err == ESP_OK) {
                    success = true;
                    strcpy(message, "停止回放");
                }
            }
        }
        
        cJSON_Delete(root);
    }
    
    cJSON_AddBoolToObject(response, "success", success);
    cJSON_AddStringToObject(response, "message", message);
    
    char *json_str = cJSON_PrintUnformatted(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json_str);
    
    free(json_str);
    cJSON_Delete(response);
    xSemaphoreGive(controller->json_mutex);
    
    return ESP_OK;
}

// 下载路径文件
static esp_err_t download_path_handler(httpd_req_t *req)
{
    struct web_controller *controller = req->user_ctx;
    char path_index_str[32];
    
    // 获取URL查询参数
    if (httpd_req_get_url_query_str(req, path_index_str, sizeof(path_index_str)) != ESP_OK) {
        ESP_LOGE(TAG, "无法获取URL查询参数");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "下载路径请求，查询参数: %s", path_index_str);
    
    char param[16];
    if (httpd_query_key_value(path_index_str, "idx", param, sizeof(param)) != ESP_OK) {
        ESP_LOGE(TAG, "无法获取idx参数");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    
    int path_index = atoi(param);
    ESP_LOGI(TAG, "请求下载路径索引: %d", path_index);
    
    if (path_index < 0 || path_index >= LEARNING_MAX_PATHS) {
        ESP_LOGE(TAG, "路径索引超出范围: %d", path_index);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    
    xSemaphoreTake(controller->json_mutex, portMAX_DELAY);
    
    // 加载路径
    esp_err_t err = learning_load_path(controller->learning, path_index);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "加载路径 %d 失败: %d", path_index, err);
        xSemaphoreGive(controller->json_mutex);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    
    // 检查路径点数量
    if (controller->learning->current_path.point_count == 0) {
        ESP_LOGE(TAG, "路径 %d 点数量为0", path_index);
        xSemaphoreGive(controller->json_mutex);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "成功加载路径 %d，点数量: %u, 总时间: %lu ms", 
             path_index, controller->learning->current_path.point_count, 
             controller->learning->current_path.total_time_ms);
    
    // 制作路径的JSON表示
    cJSON *root = cJSON_CreateObject();
    cJSON *points_array = cJSON_CreateArray();
    
    cJSON_AddNumberToObject(root, "path_index", path_index);
    cJSON_AddNumberToObject(root, "point_count", controller->learning->current_path.point_count);
    cJSON_AddNumberToObject(root, "total_time_ms", controller->learning->current_path.total_time_ms);
    
    for (int i = 0; i < controller->learning->current_path.point_count; i++) {
        path_point_t *point = &controller->learning->current_path.points[i];
        cJSON *point_obj = cJSON_CreateObject();
        
        cJSON_AddNumberToObject(point_obj, "timestamp", point->timestamp);
        
        cJSON *angles = cJSON_CreateObject();
        cJSON_AddNumberToObject(angles, "base", point->angles.base);
        cJSON_AddNumberToObject(angles, "shoulder", point->angles.shoulder);
        cJSON_AddNumberToObject(angles, "elbow", point->angles.elbow);
        cJSON_AddNumberToObject(angles, "wrist", point->angles.wrist);
        cJSON_AddNumberToObject(angles, "gripper", point->angles.gripper);
        
        cJSON_AddItemToObject(point_obj, "angles", angles);
        cJSON_AddItemToArray(points_array, point_obj);
    }
    
    cJSON_AddItemToObject(root, "points", points_array);
    
    char *json_str = cJSON_PrintUnformatted(root);
    size_t json_len = strlen(json_str);
    
    // 设置响应头
    httpd_resp_set_type(req, "application/json");
    char attachment[40];
    snprintf(attachment, sizeof(attachment), "attachment; filename=path_%d.json", path_index);
    httpd_resp_set_hdr(req, "Content-Disposition", attachment);
    
    // 发送JSON文件
    httpd_resp_send(req, json_str, json_len);
    
    free(json_str);
    cJSON_Delete(root);
    xSemaphoreGive(controller->json_mutex);
    
    return ESP_OK;
}

// 上传路径文件
static esp_err_t upload_path_handler(httpd_req_t *req)
{
    struct web_controller *controller = req->user_ctx;
    char *content = NULL;
    int content_len = req->content_len;
    
    ESP_LOGI(TAG, "接收上传路径文件请求，内容长度: %d 字节", content_len);
    
    // 检查内容长度是否合理
    if (content_len <= 0) {
        ESP_LOGE(TAG, "无效的内容长度: %d", content_len);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "无效的内容长度");
        return ESP_FAIL;
    }
    
    // 检查内容长度是否过大
    const int MAX_CONTENT_SIZE = 32768; // 32KB，根据ESP32内存情况调整
    if (content_len > MAX_CONTENT_SIZE) {
        ESP_LOGE(TAG, "内容太大: %d 字节，最大允许: %d 字节", content_len, MAX_CONTENT_SIZE);
        httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "内容太大");
        return ESP_FAIL;
    }
    
    // 动态分配内存
    content = malloc(content_len + 1);
    if (content == NULL) {
        ESP_LOGE(TAG, "内存分配失败，请求大小: %d 字节", content_len + 1);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "服务器内存不足");
        return ESP_FAIL;
    }
    
    // 接收内容
    int received = 0;
    int remaining = content_len;
    
    ESP_LOGI(TAG, "开始接收上传数据...");
    
    while (remaining > 0) {
        int recv_len = httpd_req_recv(req, content + received, remaining);
        if (recv_len <= 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "接收超时，继续尝试");
                continue; // 超时，继续尝试
            }
            ESP_LOGE(TAG, "接收失败，错误码: %d", recv_len);
            free(content);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "接收失败");
            return ESP_FAIL;
        }
        received += recv_len;
        remaining -= recv_len;
        ESP_LOGI(TAG, "已接收 %d/%d 字节", received, content_len);
    }
    content[received] = '\0';
    
    ESP_LOGI(TAG, "数据接收完成，开始处理...");
    
    xSemaphoreTake(controller->json_mutex, portMAX_DELAY);
    
    cJSON *root = cJSON_Parse(content);
    if (root == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        ESP_LOGE(TAG, "JSON解析失败: %s", error_ptr ? error_ptr : "未知错误");
        xSemaphoreGive(controller->json_mutex);
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON格式错误");
        return ESP_FAIL;
    }
    
    // 内存可以释放了，JSON已经解析到结构中
    free(content);
    content = NULL;
    
    cJSON *response = cJSON_CreateObject();
    bool success = false;
    char message[100] = "操作失败";
    
    cJSON *path_index_item = cJSON_GetObjectItem(root, "path_index");
    cJSON *points_array = cJSON_GetObjectItem(root, "points");
    
    if (!path_index_item || !cJSON_IsNumber(path_index_item)) {
        ESP_LOGE(TAG, "缺少有效的path_index字段");
        strcpy(message, "缺少有效的path_index字段");
    } else if (!points_array || !cJSON_IsArray(points_array)) {
        ESP_LOGE(TAG, "缺少有效的points数组");
        strcpy(message, "缺少有效的points数组");
    } else {
        int path_index = path_index_item->valueint;
        
        if (path_index < 0 || path_index >= LEARNING_MAX_PATHS) {
            ESP_LOGE(TAG, "路径索引超出范围: %d", path_index);
            snprintf(message, sizeof(message), "路径索引超出范围: %d，有效范围: 0-%d", 
                    path_index, LEARNING_MAX_PATHS - 1);
        } else {
            robot_path_t new_path;
            memset(&new_path, 0, sizeof(robot_path_t));
            
            int point_count = cJSON_GetArraySize(points_array);
            ESP_LOGI(TAG, "路径 %d 包含 %d 个点", path_index, point_count);
            
            if (point_count > LEARNING_MAX_PATH_POINTS) {
                ESP_LOGW(TAG, "点数量超出限制，将截断到 %d 个点", LEARNING_MAX_PATH_POINTS);
                point_count = LEARNING_MAX_PATH_POINTS;
            }
            
            new_path.point_count = point_count;
            
            for (int i = 0; i < point_count; i++) {
                cJSON *point_obj = cJSON_GetArrayItem(points_array, i);
                if (!point_obj) {
                    ESP_LOGW(TAG, "点 %d 无效，跳过", i);
                    continue;
                }
                
                cJSON *timestamp = cJSON_GetObjectItem(point_obj, "timestamp");
                cJSON *angles = cJSON_GetObjectItem(point_obj, "angles");
                
                if (!timestamp || !cJSON_IsNumber(timestamp)) {
                    ESP_LOGW(TAG, "点 %d 缺少有效的timestamp", i);
                    continue;
                }
                
                if (!angles) {
                    ESP_LOGW(TAG, "点 %d 缺少angles对象", i);
                    continue;
                }
                
                path_point_t *point = &new_path.points[i];
                point->timestamp = timestamp->valueint;
                
                // 设置默认角度值（当前角度）
                point->angles = controller->arm->current_angles;
                
                // 覆盖JSON中提供的角度值
                cJSON *base = cJSON_GetObjectItem(angles, "base");
                if (base && cJSON_IsNumber(base)) {
                    point->angles.base = base->valuedouble;
                }
                
                cJSON *shoulder = cJSON_GetObjectItem(angles, "shoulder");
                if (shoulder && cJSON_IsNumber(shoulder)) {
                    point->angles.shoulder = shoulder->valuedouble;
                }
                
                cJSON *elbow = cJSON_GetObjectItem(angles, "elbow");
                if (elbow && cJSON_IsNumber(elbow)) {
                    point->angles.elbow = elbow->valuedouble;
                }
                
                cJSON *wrist = cJSON_GetObjectItem(angles, "wrist");
                if (wrist && cJSON_IsNumber(wrist)) {
                    point->angles.wrist = wrist->valuedouble;
                }
                
                cJSON *gripper = cJSON_GetObjectItem(angles, "gripper");
                if (gripper && cJSON_IsNumber(gripper)) {
                    point->angles.gripper = gripper->valuedouble;
                }
            }
            
            // 设置路径总时间
            if (point_count > 0) {
                new_path.total_time_ms = new_path.points[point_count - 1].timestamp;
                ESP_LOGI(TAG, "路径总时间: %lu ms", new_path.total_time_ms);
            } else {
                ESP_LOGW(TAG, "路径点数量为0，总时间设为0");
                new_path.total_time_ms = 0;
            }
            
            // 复制到当前路径
            memcpy(&controller->learning->current_path, &new_path, sizeof(robot_path_t));
            
            // 保存路径
            ESP_LOGI(TAG, "开始保存路径 %d 到NVS...", path_index);
            esp_err_t err = learning_save_path(controller->learning, path_index);
            if (err == ESP_OK) {
                success = true;
                snprintf(message, sizeof(message), "路径 %d 上传并保存成功，包含 %d 个点", 
                        path_index, point_count);
                ESP_LOGI(TAG, "%s", message);
            } else {
                snprintf(message, sizeof(message), "保存路径 %d 失败，错误: %s", 
                        path_index, esp_err_to_name(err));
                ESP_LOGE(TAG, "%s", message);
            }
        }
    }
    
    cJSON_Delete(root);
    
    cJSON_AddBoolToObject(response, "success", success);
    cJSON_AddStringToObject(response, "message", message);
    
    char *json_str = cJSON_PrintUnformatted(response);
    if (!json_str) {
        ESP_LOGE(TAG, "生成响应JSON失败");
        cJSON_Delete(response);
        xSemaphoreGive(controller->json_mutex);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "生成响应失败");
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    
    free(json_str);
    cJSON_Delete(response);
    xSemaphoreGive(controller->json_mutex);
    
    ESP_LOGI(TAG, "上传路径处理完成");
    return ESP_OK;
}

// 初始化SPIFFS文件系统
static esp_err_t init_spiffs(void)
{
    ESP_LOGI(TAG, "初始化文件系统");
    
    // 配置SPIFFS文件系统
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/www",
        .partition_label = "www",
        .max_files = 5,
        .format_if_mount_failed = true
    };
    
    // 挂载SPIFFS文件系统
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "无法挂载或格式化文件系统");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "找不到指定的分区");
        } else {
            ESP_LOGE(TAG, "初始化文件系统失败: %s", esp_err_to_name(ret));
        }
        return ret;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info("www", &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "获取SPIFFS信息失败");
        return ret;
    }
    
    ESP_LOGI(TAG, "文件系统挂载成功，总大小: %d KB, 已使用: %d KB", total / 1024, used / 1024);
    return ESP_OK;
}

// 生成HTML内容 - 仅作为备用，现在主要从SPIFFS读取
static char *generate_html_content(void)
{
    char *html = malloc(MAX_HTML_SIZE);
    if (!html) {
        ESP_LOGE(TAG, "无法分配HTML内存");
        return NULL;
    }
    
    // 使用最小的HTML内容，作为备用
    strcpy(html, "<html><body><h1>ESP32 Robot Arm</h1><p>Error loading web interface from SPIFFS.</p></body></html>");
    
    return html;
}

// 初始化HTTP服务器
static esp_err_t init_http_server(struct web_controller *handle)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = DEFAULT_HTTP_PORT;
    config.lru_purge_enable = true;
    
    // 启动HTTP服务器
    ESP_LOGI(TAG, "启动HTTP服务器，端口:%d", config.server_port);
    
    esp_err_t ret = httpd_start(&handle->server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动HTTP服务器失败: %d", ret);
        return ret;
    }
    
    // 注册静态文件处理程序
    register_wifi_handlers(handle->server);
    
    // 注册API处理函数
    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = handle
    };
    httpd_register_uri_handler(handle->server, &index_uri);
    
    httpd_uri_t arm_status_uri = {
        .uri       = "/api/status",
        .method    = HTTP_GET,
        .handler   = arm_status_handler,
        .user_ctx  = handle
    };
    httpd_register_uri_handler(handle->server, &arm_status_uri);
    
    httpd_uri_t arm_control_uri = {
        .uri       = "/api/control",
        .method    = HTTP_POST,
        .handler   = arm_control_handler,
        .user_ctx  = handle
    };
    httpd_register_uri_handler(handle->server, &arm_control_uri);
    
    httpd_uri_t learning_control_uri = {
        .uri       = "/api/learning",
        .method    = HTTP_POST,
        .handler   = learning_control_handler,
        .user_ctx  = handle
    };
    httpd_register_uri_handler(handle->server, &learning_control_uri);
    
    httpd_uri_t download_path_uri = {
        .uri       = "/api/download_path",
        .method    = HTTP_GET,
        .handler   = download_path_handler,
        .user_ctx  = handle
    };
    httpd_register_uri_handler(handle->server, &download_path_uri);
    
    httpd_uri_t upload_path_uri = {
        .uri       = "/api/upload_path",
        .method    = HTTP_POST,
        .handler   = upload_path_handler,
        .user_ctx  = handle
    };
    httpd_register_uri_handler(handle->server, &upload_path_uri);
    
    ESP_LOGI(TAG, "HTTP服务器已启动，请访问: http://%s/", handle->ip_addr);
    
    return ESP_OK;
}

// 初始化网页控制器
esp_err_t web_controller_init(const web_controller_config_t *config, web_controller_handle_t *handle)
{
    if (!config || !handle || !config->arm || !config->learning) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 分配控制器结构体内存
    struct web_controller *controller = calloc(1, sizeof(struct web_controller));
    if (!controller) {
        ESP_LOGE(TAG, "分配内存失败");
        return ESP_ERR_NO_MEM;
    }
    
    controller->arm = config->arm;
    controller->learning = config->learning;
    controller->ap_mode = config->ap_mode;
    controller->wifi_event_group = xEventGroupCreate();
    
    // 创建互斥锁
    controller->json_mutex = xSemaphoreCreateMutex();
    if (!controller->json_mutex) {
        ESP_LOGE(TAG, "创建互斥锁失败");
        free(controller);
        return ESP_ERR_NO_MEM;
    }
    
    // 初始化SPIFFS文件系统
    esp_err_t ret = init_spiffs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初始化SPIFFS失败");
        // 继续执行，使用备用HTML内容
    }
    
    // 生成HTML内容
    controller->html_content = generate_html_content();
    if (!controller->html_content) {
        ESP_LOGE(TAG, "生成HTML内容失败");
        vSemaphoreDelete(controller->json_mutex);
        free(controller);
        return ESP_ERR_NO_MEM;
    }
    
    *handle = controller;
    return ESP_OK;
}

// 启动网页控制器
esp_err_t web_controller_start(web_controller_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    struct web_controller *controller = (struct web_controller *)handle;
    
    // 初始化ESP-NETIF
    ESP_ERROR_CHECK(esp_netif_init());
    
    // 创建默认事件循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // 创建WiFi STA或AP接口
    if (controller->ap_mode) {
        esp_netif_create_default_wifi_ap();
    } else {
        esp_netif_create_default_wifi_sta();
    }
    
    // 初始化WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // 注册WiFi事件处理函数
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                       ESP_EVENT_ANY_ID,
                                                       &wifi_event_handler,
                                                       controller,
                                                       &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                       IP_EVENT_STA_GOT_IP,
                                                       &wifi_event_handler,
                                                       controller,
                                                       &instance_got_ip));
    
    esp_err_t ret = ESP_OK;
    
    // 启动WiFi
    if (controller->ap_mode) {
        ret = init_wifi_ap(controller);
    } else {
        ret = init_wifi_sta(controller, ESP32_WIFI_SSID, ESP32_WIFI_PASSWORD);
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动WiFi失败: %d", ret);
        return ret;
    }
    
    // 启动HTTP服务器
    ret = init_http_server(controller);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动HTTP服务器失败: %d", ret);
        return ret;
    }
    
    return ESP_OK;
}

// 停止网页控制器
esp_err_t web_controller_stop(web_controller_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    struct web_controller *controller = (struct web_controller *)handle;
    
    // 停止HTTP服务器
    if (controller->server) {
        httpd_stop(controller->server);
    }
    
    // 停止WiFi
    esp_wifi_stop();
    esp_wifi_deinit();
    
    // 释放资源
    if (controller->html_content) {
        free(controller->html_content);
    }
    
    if (controller->json_mutex) {
        vSemaphoreDelete(controller->json_mutex);
    }
    
    if (controller->wifi_event_group) {
        vEventGroupDelete(controller->wifi_event_group);
    }
    
    free(controller);
    
    return ESP_OK;
}

// 获取IP地址
const char *web_controller_get_ip(web_controller_handle_t handle)
{
    if (!handle) {
        return NULL;
    }
    
    struct web_controller *controller = (struct web_controller *)handle;
    return controller->ip_addr;
} 