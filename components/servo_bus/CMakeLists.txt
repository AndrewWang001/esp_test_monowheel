idf_component_register(SRCS "servo.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_uart esp_driver_gpio pins
                       PRIV_REQUIRES esp_timer)
