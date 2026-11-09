idf_component_register(SRCS "main.cpp"
    INCLUDE_DIRS "." "../../../rcmultiswitchG030" "../../../../include_stm32"
    PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_driver_gptimer esp_timer nvs_flash)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_23)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    USE_MORSE USE_OPERATE_MENU USE_FAILSAFE USE_RESET_COMMAND USE_TELEMETRY)
