#include <iusl_uav/iusl_camera_node.h>

using namespace iusl_camera;

void CameraHandler::imageSubCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat cv_img = cv_bridge::toCvShare(msg, "bgr8")->image; 
    if (cv_img.empty()) return; 
    /* ---  cv img crop, using the center of height --- */
    int cv_img_width = cv_img.cols;
    int cv_img_height = cv_img.rows;
    int cv_expect_height = cv_img_width/16*9;
    if (cv_img_height > cv_expect_height)
    {
        cv::Rect myROI(0, (cv_img_height-cv_expect_height)/2, cv_img_width, cv_expect_height);
        cv_img = cv_img(myROI);
    }
    _cv_img = cv_img;

}

void CameraHandler::imageDisplay()
{
    if (_cv_img.empty()) return; 
    cv::imshow("view", _cv_img);
    cv::waitKey(10);
}