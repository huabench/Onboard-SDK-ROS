#ifndef _IUSL_CAMERA_NODE_
#define _IUSL_CAMERA_NODE_

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h> 
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <memory>
/* USER DEFINED LIB */
#include <dji_osdk_ros/SetupCameraStream.h>
#include <iusl_uav/iusl_image_handler.h>

namespace iusl_camera {

    using namespace dji_osdk_ros;
    using namespace iusl_image_handler;

    using uniptr_image_handler = std::unique_ptr<ImageHandler>;

    enum class CameraCMD {
        START = 1,
        STOP = 0
    };

    class CameraHandler {
        public:
            CameraHandler();
            ~CameraHandler();

            void imageDisplay();
            static void MySigintHanlder(int sig);
            static void setCamera(CameraCMD);


        private:
            ros::NodeHandle _nh;
            image_transport::ImageTransport _it;
            image_transport::Subscriber _it_sub;
            uniptr_image_handler _image_handler;
            
            void imageSubCallback(const sensor_msgs::ImageConstPtr& msg);
    };

    //using uniptr_camera_handler = std::unique_ptr<CameraHandler>;


#endif   
}