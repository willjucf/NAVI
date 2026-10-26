idf_component_register(
    SRCS
        "main.cpp"
        "safety.cpp"
        "sensors.cpp"
        "navi_logic.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_timer
)
