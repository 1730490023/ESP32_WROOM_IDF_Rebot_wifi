idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash 
             esp_netif 
             esp_wifi 
             esp_http_server 
             spiffs
             robotic_arm
             learning_controller
             web_controller
)

# 添加分区表注释
message(STATUS "使用自定义分区表: partitions.csv")
