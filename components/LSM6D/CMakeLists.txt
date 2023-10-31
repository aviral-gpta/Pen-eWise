set(COMPONENT_SRCS
    "LSM6D.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_REQUIRES
    driver
    I2Cbus
)

register_component()