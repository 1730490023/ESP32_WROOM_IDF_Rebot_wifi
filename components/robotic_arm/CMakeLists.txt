idf_component_register(
    SRCS "robotic_arm.c"
    INCLUDE_DIRS "include"
    REQUIRES servo_controller esp_timer freertos
)