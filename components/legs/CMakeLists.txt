idf_component_register(SRCS "legs.c"
                       INCLUDE_DIRS "include"
                       REQUIRES leg_kinematics
                       PRIV_REQUIRES servo_bus)
