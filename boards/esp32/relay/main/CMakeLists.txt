idf_component_register(SRCS "main.cpp"
    INCLUDE_DIRS "." "../../../rcRelay32" "../../../../include_stm32"
    PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_timer nvs_flash esp_system)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_23)
