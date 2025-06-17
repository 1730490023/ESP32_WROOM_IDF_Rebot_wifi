idf_component_register(
    SRCS "pca9685.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)