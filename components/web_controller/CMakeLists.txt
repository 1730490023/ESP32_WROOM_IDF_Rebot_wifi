idf_component_register(SRCS "web_controller.c" "wifi_handlers.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_server esp_wifi nvs_flash learning_controller robotic_arm json spiffs fatfs)

# 创建一个目标来将HTML文件嵌入到SPIFFS分区
spiffs_create_partition_image(www ${CMAKE_CURRENT_SOURCE_DIR}/www FLASH_IN_PROJECT) 