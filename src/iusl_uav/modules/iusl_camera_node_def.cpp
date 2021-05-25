#include <iusl_uav/iusl_camera_node.h>

using namespace iusl_camera;

void CameraHandler::setCamera(CameraCMD cmd) {
    ros::NodeHandle _nh_t;  // static function cannot access class member
    ros::service::waitForService("/setup_camera_stream");
    ros::ServiceClient set_camera_client = _nh_t.serviceClient<dji_osdk_ros::SetupCameraStream>("/setup_camera_stream");
    dji_osdk_ros::SetupCameraStream set_camera_srv;
    set_camera_srv.request.cameraType = 1;  // 1 is for main camera
    set_camera_srv.request.start = (uint8_t) cmd;
    set_camera_client.call(set_camera_srv);
}

CameraHandler::CameraHandler(): _it(_nh), _image_handler(new ImageHandler()) {
    // start camera
    CameraHandler::setCamera(CameraCMD::START);
    // subscribe camera image
    _it_sub = _it.subscribe("/iusl/main_camera_images", 1, &CameraHandler::imageSubCallback, this);
}

CameraHandler::~CameraHandler() {
    CameraHandler::setCamera(CameraCMD::STOP);
}


void CameraHandler::imageSubCallback(const sensor_msgs::ImageConstPtr& msg) {
    _image_handler->updateImage(msg);
    _image_handler->detectAndTrack();
}

void CameraHandler::imageDisplay() {
    _image_handler->imageDisplay();
}

