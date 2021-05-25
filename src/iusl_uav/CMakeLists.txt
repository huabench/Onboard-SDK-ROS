ADD_EXECUTABLE(iusl_vehicle_node iusl_vehicle_node.cpp)
target_link_libraries(iusl_vehicle_node ${catkin_LIBRARIES})

TARGET_LINK_LIBRARIES(iusl_vehicle_node
                    ${PROJECT_NAME}_extend
                    )

find_package( OpenCV 3.3.0 QUIET )
    if(OpenCV_FOUND)
    add_definitions(-DOPENCV)
endif()

FILE(GLOB FILES modules/*.cpp)

ADD_EXECUTABLE(iusl_camera_node
    ${FILES}
    iusl_camera_node.cpp 
)


target_link_libraries(
    iusl_camera_node
    ${PROJECT_SOURCE_DIR}/src/iusl_uav/darknet/libdarknet.so
    ${PROJECT_NAME} 
    ${catkin_LIBRARIES}
)

