# set(COMPONENT_SRCS
#     "LSM6D.hpp")
# set(COMPONENT_ADD_INCLUDEDIRS "include")
# set(COMPONENT_REQUIRES
#     driver
#     I2Cbus
# )

# register_component()

idf_component_register(SRCS "LSM6D.cpp"
                  INCLUDE_DIRS "include" "."
                  REQUIRES driver I2Cbus)