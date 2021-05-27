#include <iusl_uav/iusl_camera_node.h>
#include <dji_osdk_ros/common_type.h>

using namespace iusl_uav;

CameraHandler::CameraHandler(): 
    _nh(), 
    _it(_nh), 
    _it_sub (_it.subscribe("/iusl/main_camera_images", 1, &CameraHandler::imageSubCallback, this)),
    _image_handler(new ImageHandler()),
    _msg_handler(new MessageHandler()),
    _gim_max_speed(6),
    _gim_control_k(0.015),
    _gimbalAction()
{
    CameraHandler::setCamera(CameraCMD::START);
}

CameraHandler::~CameraHandler() {
    // stop camera when stopped
    CameraHandler::setCamera(CameraCMD::STOP);
}

void CameraHandler::setCamera(CameraCMD cmd) const{
    ros::NodeHandle _nh_t;  // static function cannot access class member
    ros::service::waitForService("/setup_camera_stream");
    ros::ServiceClient set_camera_client = _nh_t.serviceClient<dji_osdk_ros::SetupCameraStream>("/setup_camera_stream");
    dji_osdk_ros::SetupCameraStream set_camera_srv;
    set_camera_srv.request.cameraType = 1;  // 1 is for main camera
    set_camera_srv.request.start = (uint8_t) cmd;
    set_camera_client.call(set_camera_srv);

    ROS_INFO("Enter setCamera");
}


/* =================== Private functino =================== */

void CameraHandler::imageSubCallback(const sensor_msgs::ImageConstPtr& msg) {
    _image_handler->updateImage(msg);
    _image_handler->detectAndTrack();
}

void CameraHandler::imageDisplay() const{
    _image_handler->imageDisplay();
}

void CameraHandler::sendData() const{
    _msg_handler->publishDetectionResult(_image_handler->getDetectBoxInfo());
}

void CameraHandler::rotateGimbal() {
    const DetectionBoxInfo box_info = _image_handler->getDetectBoxInfo();
    const VehicleStatus vehicle_state = _msg_handler->getVehicleStatus();

    int net_width = _image_handler->getNetWidth();
    int net_height =  _image_handler->getNetHeight();
    int dx = net_width/2 - box_info.center_x;
    int dy = net_height/2 - box_info.center_y;

    float gim_speed_q = dy * _gim_control_k;
    float gim_speed_r = - dx * _gim_control_k;

    if (gim_speed_q > _gim_max_speed) {
        gim_speed_q = _gim_max_speed;
    } else if (gim_speed_q < -_gim_max_speed) {
        gim_speed_q = -_gim_max_speed;
    }

    if (gim_speed_r > _gim_max_speed) {
        gim_speed_r = _gim_max_speed;
    }
    else if (gim_speed_r < -_gim_max_speed) {
        gim_speed_r = -_gim_max_speed;
    }

    _gimbalAction.request.is_reset = false;
    _gimbalAction.request.payload_index = static_cast<uint8_t>(PayloadIndex::PAYLOAD_INDEX_0);
    _gimbalAction.request.rotationMode = 0;
    _gimbalAction.request.pitch = gim_speed_q + vehicle_state.gim_pitch_now;
    _gimbalAction.request.roll = 0.0f;
    _gimbalAction.request.yaw = gim_speed_r + vehicle_state.gim_yaw_now;
    _gimbalAction.request.time = std::max(dx, dy)/800 + 1;

    _msg_handler->publishGimbalData(_gimbalAction);
}