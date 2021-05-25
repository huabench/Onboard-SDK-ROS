#include <iusl_uav/iusl_camera_node.h>
#include <signal.h>

using namespace iusl_camera;


/*FIXME: run CameraHandler destrcutor instead of 
    making this function static
*/

void MySigintHanlder(int sig) { 
    ROS_INFO("Shutting down this node!");
/*
    CameraHandler *p = camera_handler.release();
    if(camera_handler == nullptr) {
        ROS_INFO("camera handler released \n");
    }
    delete p;
*/
    CameraHandler::setCamera(CameraCMD::STOP);;
    ros::shutdown();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "iusl_cameara_node");
    //uniptr_camera_handler camera_handler(new CameraHandler);
    CameraHandler camera_handler;
    ros::Rate rate(50);
    signal(SIGINT, MySigintHanlder);
   
    while(ros::ok()) {
        //camera_handler->imageDisplay();
        camera_handler.imageDisplay();
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("stop ===========================]");
    return 0;
}