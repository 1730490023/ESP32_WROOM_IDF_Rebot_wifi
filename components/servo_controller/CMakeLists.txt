idf_component_register(
    SRCS "servo_controller.c"
    INCLUDE_DIRS "include"
    REQUIRES pca9685 esp_timer esp_common
)