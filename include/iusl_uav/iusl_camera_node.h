#ifndef _IUSL_CAMERA_NODE_
#define _IUSL_CAMERA_NODE_

#include <image_transport/image_transport.h>
/* USER DEFINED LIB */
#include <dji_osdk_ros/SetupCameraStream.h>
#include <iusl_uav/iusl_image_handler.h>
#include <iusl_uav/iusl_message_handler.h>
#include <dji_osdk_ros/GimbalAction.h>


namespace iusl_uav {
    using namespace dji_osdk_ros;
    using uniptr_image_handler = std::unique_ptr<ImageHandler>;
    using uniptr_message_handler = std::unique_ptr<MessageHandler>;  

    enum class CameraCMD {
        START = 1,
        STOP = 0
    };

    class CameraHandler {
        public:
            CameraHandler();
            ~CameraHandler();

            void imageDisplay() const;
            void rotateGimbal();
            void sendData() const; // TODO: change name to what
            void setCamera(CameraCMD) const;


        private:
            ros::NodeHandle _nh;
            image_transport::ImageTransport _it;
            image_transport::Subscriber _it_sub;
            uniptr_image_handler _image_handler;
            uniptr_message_handler _msg_handler;

            const int _gim_max_speed;
            const float _gim_control_k;
            GimbalAction _gimbalAction;
            
            void imageSubCallback(const sensor_msgs::ImageConstPtr& msg);
    };


#endif   
}