# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${SAI2-EXAMPLES_BINARY_DIR}/03-two_arms)
ADD_EXECUTABLE (controller03 controller.cpp)
ADD_EXECUTABLE (simviz03 simviz.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller03 ${SAI2-EXAMPLES_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz03 ${SAI2-EXAMPLES_COMMON_LIBRARIES})

# copy example interface to output
FILE(COPY 03-two_arms.html DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf panda_arm.urdf panda_arm_flat_ee.urdf DESTINATION ${APP_RESOURCE_DIR})

# copy in sai2-interfaces 
FILE(COPY ${SAI2-INTERFACES_UI_DIR} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})