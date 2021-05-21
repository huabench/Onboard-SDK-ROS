#include <iusl_uav/iusl_camera_node.h>

using namespace iusl_camera;

CameraHandler::CameraHandler(): _it(_nh){
    // send request and setup camera
    ROS_INFO("[iusl_camera_node] wait for service");
    ros::service::waitForService("/setup_camera_stream");
    ros::ServiceClient set_camera_client = _nh.serviceClient<dji_osdk_ros::SetupCameraStream>("/setup_camera_stream");
    dji_osdk_ros::SetupCameraStream set_camera_srv;
    set_camera_srv.request.cameraType = 1;
    set_camera_srv.request.start = 1;
    ROS_INFO("[iusl_camera_node] enter call service");
    set_camera_client.call(set_camera_srv);
    // subscribe camera image
    _it_sub = _it.subscribe("/iusl/main_camera_images", 1, &CameraHandler::imageSubCallback, this);
    // init wcv window
    cv::namedWindow("view");
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "iusl_cameara_node");

    CameraHandler camera_handler;

    ros::Rate rate(50);
   
    ROS_INFO("[iusl_camera_node] enter and finish init");

    while(ros::ok()) {
        camera_handler.imageDisplay();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}