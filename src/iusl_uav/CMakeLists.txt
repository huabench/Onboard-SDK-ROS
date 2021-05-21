ADD_EXECUTABLE(iusl_vehicle_node iusl_vehicle_node.cpp)
target_link_libraries(iusl_vehicle_node ${catkin_LIBRARIES})

TARGET_LINK_LIBRARIES(iusl_vehicle_node
                    ${PROJECT_NAME}_extend
                    )

find_package( OpenCV 3.3.0 QUIET )
    if(OpenCV_FOUND)
    add_definitions(-DOPENCV)
endif()


ADD_EXECUTABLE(iusl_camera_node iusl_camera_node.cpp modules/iusl_camera_node_image_handler.cpp)
target_link_libraries(
    iusl_camera_node
    ${PROJECT_NAME} 
    ${catkin_LIBRARIES}
)
