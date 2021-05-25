#ifndef _ISUL_VEHICLE_NODE_
#define _ISUL_VEHICLE_NODE_

#include <dji_osdk_ros/dji_vehicle_node.h>
#include <image_transport/image_transport.h>


namespace iusl_uav {
    using namespace dji_osdk_ros;

    class IUSLVehicleNode : protected VehicleNode {
        public:
           IUSLVehicleNode();
           ~IUSLVehicleNode() {}

        private:
            ros::NodeHandle _nh;
            image_transport::ImageTransport _it;
            image_transport::Publisher _image_transport_pub;
            void publishMainCameraImageCore(sensor_msgs::Image) override;

    };
}

#endif