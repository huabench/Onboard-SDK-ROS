#include <iusl_uav/iusl_message_handler.h>

using namespace iusl_uav;

MessageHandler::MessageHandler():
    _vehicle_status(),
    _nh(),
    _sub_rtk_state(_nh.subscribe("/dji_osdk_ros/rtk_connection_status", 
        5, &MessageHandler::rtkStateCallback, this)),
    _sub_rtk_pos(_nh.subscribe("/dji_osdk_ros/rtk_position", 
        5, &MessageHandler::rtkPosCallback, this)),
    _sub_rtk_vel(_nh.subscribe("/dji_osdk_ros/rtk_velocity", 
        5, &MessageHandler::rtkVelCallback, this)),
    _sub_gim_angle(_nh.subscribe("/dji_osdk_ros/gimbal_angle", 
        5, &MessageHandler::gimbalAngleCallback, this)),
    _sub_mobile(_nh.subscribe("/dji_osdk_ros/from_mobile_data",
        5, &MessageHandler::mobileCallback, this)),
    _sub_flight_state(_nh.subscribe("/dji_osdk_ros/flight_status", 
        5, &MessageHandler::flightStateCallback, this)),
    _detectionResultPublisher(_nh.advertise<DJI::iuslDetectionResult>("/iusl/DetectionResult", 1)),
    _gimbal_control_client(_nh.serviceClient<DJI::GimbalAction>("gimbal_task_control"))
    //_mobileBoxPublisher(_nh.advertise<std_msgs::Int32MultiArray>("/iusl/mobile_box", 5))

{
    ROS_INFO("Message Handler init!");
}

MessageHandler::~MessageHandler() {
    
}

const VehicleStatus& MessageHandler::getVehicleStatus() const {
    return _vehicle_status;
}


void MessageHandler::publishGimbalData(DJI::GimbalAction& gimbal_action) {
    ROS_INFO("Gimbal Contriol Data: \n is_rest: %d\n rotatoionMode: %d\n pitch: %f\n roll: %f \n yaw: %f\n t: %f\n",
        gimbal_action.request.is_reset, gimbal_action.request.rotationMode,
        gimbal_action.request.pitch, gimbal_action.request.roll,
        gimbal_action.request.yaw, gimbal_action.request.time);

    _gimbal_control_client.call(gimbal_action);
}

void MessageHandler::publishDetectionResult(const DetectionBoxInfo& box_info) const {
    DJI::iuslDetectionResult res;
    res.center_x = box_info.center_x;
    res.center_y = box_info.center_y;
    res.max_length = box_info.max_length;

    res.pitch = _vehicle_status.gim_pitch_now;
    res.yaw = _vehicle_status.gim_yaw_now;
    res.roll = _vehicle_status.gim_roll_now;
    res.UAV_lat = _vehicle_status.UAV_lat_now;
    res.UAV_lon = _vehicle_status.UAV_lon_now;
    res.UAV_alt = _vehicle_status.UAV_alt_now;
    res.UAV_vx = _vehicle_status.UAV_vx_now;
    res.UAV_vy = _vehicle_status.UAV_vy_now;
    res.laser_dis = _vehicle_status.laser_dis_now;

    _detectionResultPublisher.publish(res);
    // TODO: add publish
}

/* vehicle state callback functions */

void MessageHandler::rtkStateCallback(const std_msgs::UInt8& msg) {
    ROS_INFO("==================== rtk status", msg.data);
}

void MessageHandler::rtkPosCallback(const sensor_msgs::NavSatFix& msg) {
    _vehicle_status.UAV_lat_now = msg.latitude;
    _vehicle_status.UAV_lon_now = msg.longitude;
    _vehicle_status.UAV_alt_now = msg.altitude;
}

void MessageHandler::rtkVelCallback(const geometry_msgs::Vector3Stamped& msg) {
    _vehicle_status.UAV_vx_now = msg.vector.y;
    _vehicle_status.UAV_vy_now = msg.vector.x;
}

void MessageHandler::gimbalAngleCallback(const geometry_msgs::Vector3Stamped& msg) {
    _vehicle_status.gim_pitch_now = msg.vector.x;
    _vehicle_status.gim_yaw_now = msg.vector.z;
    _vehicle_status.gim_roll_now = msg.vector.y;
}

void MessageHandler::mobileCallback(const DJI::MobileData& msg) {
    int data_length = sizeof(msg.data)/sizeof(msg.data[0]); 
    if (data_length != 0)
    {
        _vehicle_status.laser_dis_now = msg.data[0]*256 + msg.data[1] + (float)(msg.data[2]*256 + msg.data[3])/100;
        //ROS_INFO("Receive Data from MSDK and length of data is %02d.", data_length);
    }
}

void MessageHandler::flightStateCallback(const std_msgs::UInt8& msg) {
    _vehicle_status.flight_state = msg.data;
}