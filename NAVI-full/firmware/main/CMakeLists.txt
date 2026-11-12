idf_component_register(
    SRCS
        "main.cpp"
        "commands.cpp"
        "safety.cpp"
        "sensors.cpp"
        "navi_logic.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_ledc
        esp_driver_uart
        esp_driver_i2c
        esp_timer
)
