# Speaker clips from tools/make_sounds.py. sounds/voice.raw (16 kHz) is optional.
set(clips
    "sounds/estop.raw"
    "sounds/obstacle.raw"
    "sounds/red.raw"
    "sounds/tilt.raw"
    "sounds/link.raw"
    "sounds/resume.raw")
set(has_voice FALSE)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sounds/voice.raw")
    list(APPEND clips "sounds/voice.raw")
    set(has_voice TRUE)
endif()

idf_component_register(
    SRCS
        "main.cpp"
        "safety.cpp"
        "sensors.cpp"
        "navi_logic.cpp"
        "wireless.cpp"
        "sound.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_ledc
        esp_driver_gptimer
        esp_driver_uart
        esp_driver_i2c
        esp_timer
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        lwip
        bt
        esp_ringbuf
        app_update
    EMBED_FILES ${clips}
)

if(has_voice)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE NAVI_HAS_VOICE=1)
endif()
