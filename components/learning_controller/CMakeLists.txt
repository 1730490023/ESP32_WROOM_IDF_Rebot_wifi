idf_component_register(
    SRCS "learning_controller.c"
    INCLUDE_DIRS "include"
    REQUIRES robotic_arm nvs_flash driver esp_adc
) 