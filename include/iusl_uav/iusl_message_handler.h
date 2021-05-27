#ifndef _IUSL_MESSAGE_HANDLER_
#define _IUSL_MESSAGE_HANDLER_

#include <ros/ros.h>
#include <dji_osdk_ros/iuslDetectionResult.h>                 
#include <dji_osdk_ros/MobileData.h>            
#include <dji_osdk_ros/GimbalAction.h>
#include <std_msgs/Int32MultiArray.h> 
#include <std_msgs/UInt8.h>  
#include <geometry_msgs/Vector3Stamped.h>   
#include <sensor_msgs/NavSatFix.h>
#include <iusl_uav/iusl_common_type.h>

namespace iusl_uav {
    namespace DJI = dji_osdk_ros;

    class MessageHandler {
        public:
            MessageHandler();
            ~MessageHandler();

            void publishGimbalData(DJI::GimbalAction&);
            void publishDetectionResult(const DetectionBoxInfo&) const;

            const VehicleStatus& getVehicleStatus() const;

        private:
            VehicleStatus _vehicle_status; 

            ros::NodeHandle _nh;
            ros::Subscriber _sub_rtk_state;
            ros::Subscriber _sub_rtk_pos;
            ros::Subscriber _sub_rtk_vel;
            ros::Subscriber _sub_gim_angle;
            ros::Subscriber _sub_mobile;
            ros::Subscriber _sub_flight_state;

            ros::Publisher _detectionResultPublisher;
            ros::ServiceClient _gimbal_control_client;


            //ros::Publisher _mobileBoxPublisher;

             /* CALLBACK FUNCTION: vehicle state */
            void rtkStateCallback(const std_msgs::UInt8&);
            void rtkPosCallback(const sensor_msgs::NavSatFix&);
            void rtkVelCallback(const geometry_msgs::Vector3Stamped&);
            void gimbalAngleCallback(const geometry_msgs::Vector3Stamped&);
            void mobileCallback(const dji_osdk_ros::MobileData&);
            void flightStateCallback(const std_msgs::UInt8&);
    };
}



#endif