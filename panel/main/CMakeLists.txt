idf_component_register(
    SRCS 
        "main.cpp" 
        "../../common/espnow.c" 
        # "$ENV{HYPERION_LIB_DIR}/platform/esp32/pwmDriver.cpp"
        # "$ENV{HYPERION_LIB_DIR}/platform/esp32/log.cpp"
    INCLUDE_DIRS 
        "."
        # "$ENV{HYPERION_LIB_DIR}/platform/includes/"
        # "$ENV{HYPERION_LIB_DIR}/platform/esp32/"
)
