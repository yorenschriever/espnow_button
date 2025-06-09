idf_component_register(SRCS "deep_sleep_example_main.c"
                            "gpio_wakeup.c"
                            "ext_wakeup.c"
                            "touch_wakeup.c"
                            "../../common/espnow.c"
                    INCLUDE_DIRS ".")
