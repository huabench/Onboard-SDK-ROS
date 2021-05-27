#ifndef _ISUL_VEHICLE_NODE_
#define _ISUL_VEHICLE_NODE_

#include <image_transport/image_transport.h>
#include <dji_osdk_ros/dji_vehicle_node.h>
#include <dji_osdk_ros/iuslSetRtkEnable.h>
#include <dji_osdk_ros/Gimbal.h>


namespace iusl_uav {
    namespace DJI = dji_osdk_ros;
    using iuslSrvReq = DJI::iuslSetRtkEnable::Request&;
    using iuslSrvRes = DJI::iuslSetRtkEnable::Response&;

    class IUSLVehicleNode : protected DJI::VehicleNode {
        public:
           IUSLVehicleNode();
           ~IUSLVehicleNode() {}

        private:
            ros::NodeHandle _nh;
            image_transport::ImageTransport _it;
            image_transport::Publisher _image_transport_pub;
            ros::ServiceServer _set_rtk_enable_server;
            ros::Subscriber _iusl_gimbal_subscriber;

            bool iuslSetRtkEnableCallback(iuslSrvReq, iuslSrvRes);
            void publishMainCameraImageCore(sensor_msgs::Image) override;
            //void iuslGimbalCallback(const dji_osdk_ros::gimbal&)   //TODO: test if this is more stable

    };
}

#endif