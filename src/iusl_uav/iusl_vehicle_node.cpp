#include <iusl_uav/iusl_vehicle_node.h>

using namespace iusl_uav;

IUSLVehicleNode::IUSLVehicleNode():
    _nh(),
    _it(_nh),
    _image_transport_pub(_it.advertise("iusl/main_camera_images", 1)),
    _set_rtk_enable_server(_nh.advertiseService("/iusl/set_rtk_enable", 
        &IUSLVehicleNode::iuslSetRtkEnableCallback, this))
    {}

bool IUSLVehicleNode::iuslSetRtkEnableCallback(iuslSrvReq req, iuslSrvRes res) {
    res.result = ptr_wrapper_->getVehicle()->flightController->setRtkEnableSync(FlightController::RtkEnabled::RTK_ENABLE, 1);
    return true;
}

/* use image_transport to publish images */
void IUSLVehicleNode::publishMainCameraImageCore(sensor_msgs::Image img) {
    _image_transport_pub.publish(img);
}

/*
void IUSLVehicleNode::iuslGimbalCallback(const dji_osdk_ros::iuslGimbalCmd &msg)   //iusl ningzian, gimbalcontrol
{
  GimbalRotationData gimCmd;
  gimCmd.rotationMode = msg.mode;
  gimCmd.pitch = msg.pitch;
  gimCmd.roll = msg.roll;
  gimCmd.yaw = msg.yaw;
  gimCmd.time = msg.time;
  
  ptr_wrapper_ -> rotateGimbal(static_cast<PayloadIndex>(0), gimCmd);
}
*/


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_node");
    IUSLVehicleNode vh_node;
    
    ros::spin();
    return 0;
}


