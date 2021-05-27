#include <iusl_uav/iusl_image_detector.h>
#include <ros/package.h>
#include <ros/ros.h>

using namespace iusl_uav;

ImageDetector::ImageDetector():
    _pkg_path(ros::package::getPath("dji_osdk_ros") + "/src/iusl_uav/darknet"),
    _cv_data_file(_pkg_path + "/obj.data"),
    _cv_cfg_file(_pkg_path + "/yolov4-tiny-obj.cfg"),
    _cv_weight_file(_pkg_path + "/yolov4-tiny-obj.weights"),
    _detector(Detector(_cv_cfg_file, _cv_weight_file, 0)),   // darknet detector
    _tracker(),
    _tracker_params(),
    _detect_box(cv::Rect2d(0,0,1,1)),
    _detect_prob(0.),
    _detector_state(DetectorState::NOTHING),
    _detector_threshold(0.6),
    _detect_box_info(),
    _net_width(_detector.get_net_width()),
    _net_height(_detector.get_net_height())
{
    ROS_INFO("Message Detector init!");
}

ImageDetector::~ImageDetector() {
    
}

bool ImageDetector::detect(const cv::Mat& img) {
    auto bboxt_lists = _detector.detect(img, ImageDetector::_detector_threshold);
    bool is_detected = bboxt_lists.size() > 0;
    if (is_detected)
    { 
        _detect_box.x = bboxt_lists[0].x;
        _detect_box.y = bboxt_lists[0].y;
        _detect_box.width = bboxt_lists[0].w;
        _detect_box.height = bboxt_lists[0].h;
        _detect_prob = bboxt_lists[0].prob;

        _detect_box_info.center_x = _detect_box.x + _detect_box.width/2;
        _detect_box_info.center_y = _detect_box.y + _detect_box.height/2;
        _detect_box_info.max_length = std::max(_detect_box.width, _detect_box.height);
    }
    return is_detected;
}

const DetectionBoxInfo& ImageDetector::getDetectBoxInfo() const {
    return _detect_box_info;
}

const int& ImageDetector::getNetWidth() const {
    return _net_width;
}

const int& ImageDetector::getNetHeight() const {
    return _net_height;
}

const cv::Rect2d& ImageDetector::getDetectBox() const {
    return _detect_box;
}

const float& ImageDetector::getDetectProb() const {
    return _detect_prob;
}