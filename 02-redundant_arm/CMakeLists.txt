# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${SAI2-EXAMPLES_BINARY_DIR}/02-redundant_arm)
ADD_EXECUTABLE (02-redundant_arm 02-redundant_arm.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (02-redundant_arm ${SAI2-EXAMPLES_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf kuka_iiwa.urdf DESTINATION ${APP_RESOURCE_DIR})
