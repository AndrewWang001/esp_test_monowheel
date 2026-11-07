# The BNO085 library (rinku404/bno085) is pulled in by idf_component.yml in this folder
idf_component_register(SRCS "imu.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_i2c esp_driver_gpio pins)
