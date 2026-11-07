idf_component_register(SRCS "esc.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_ledc pins)
