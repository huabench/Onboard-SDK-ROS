#include <iusl_uav/iusl_vehicle_node.h>

using namespace iusl_uav;

IUSLVehicleNode::IUSLVehicleNode():_it(_nh) {
    _image_transport_pub = _it.advertise("iusl/main_camera_images", 1);
}
/*
void IUSLVehicleNode::publishMainCameraImageCore(sensor_msgs::Image img) {
    //_image_transport_pub.publish(img);
    VehicleNode::publishMainCameraImageCore(img);
    ROS_INFO("=================");
}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_node");
    IUSLVehicleNode vh_node;
    
    ros::spin();
    return 0;
}





