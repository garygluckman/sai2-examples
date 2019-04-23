# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${SAI2-EXAMPLES_BINARY_DIR}/01-non_redundant_arm)
ADD_EXECUTABLE (controller01 controller.cpp)
ADD_EXECUTABLE (simviz01 simviz.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller01	${SAI2-EXAMPLES_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz01	${SAI2-EXAMPLES_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf puma.urdf DESTINATION ${APP_RESOURCE_DIR})
