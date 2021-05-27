#include <iusl_uav/iusl_camera_node.h>
#include <signal.h>

using namespace iusl_uav;

static volatile bool keep_running = true;

void MySigintHanlder(int sig) { 
    ROS_INFO("Shutting down this node!");
    keep_running = false;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "iusl_cameara_node");
    CameraHandler camera_handler;
    ros::Rate rate(50);
    signal(SIGINT, MySigintHanlder);
   
    while(ros::ok() && keep_running) {
        camera_handler.imageDisplay();
        camera_handler.rotateGimbal();
        camera_handler.sendData();
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("stop ===========================]");
    return 0;
}