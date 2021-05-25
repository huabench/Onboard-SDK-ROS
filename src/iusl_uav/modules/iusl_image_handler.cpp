#include <iusl_uav/iusl_image_handler.h>

using namespace iusl_image_handler;

/* ==================== Constructor and Destructor ====================*/
ImageHandler::ImageHandler():
    pkg_path(ros::package::getPath("dji_osdk_ros") + "/src/iusl_uav/darknet"),
    _cv_data_file(pkg_path + "/obj.data"),
    _cv_cfg_file(pkg_path + "/yolov4-tiny-obj.cfg"),
    _cv_weight_file(pkg_path + "/yolov4-tiny-obj.weights"),
    _detector(Detector(_cv_cfg_file, _cv_weight_file, 0)),   // darknet detector
    _detect_box(cv::Rect2d(0,0,1,1)),
    _detect_prob(0.),
    _detector_state(DetectorState::NOTHING),
    _detector_threshold(0.6)
{
    _cv_window_name = "view";
    cv::namedWindow(_cv_window_name);
}

ImageHandler::~ImageHandler() {
    cv::destroyWindow(_cv_window_name);
}


/* ==================== Public Functions ====================*/

void ImageHandler::updateImage(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat cv_img = cv_bridge::toCvShare(msg, "bgr8")->image; 
    if (cv_img.empty()) return; 

    resizeImage(cv_img);
}

void ImageHandler::detectAndTrack() {
    detect();
}


void ImageHandler::imageDisplay()
{
    if (_cv_img.empty()) return;
    
    drawBox();

    cv::imshow("view", _cv_img);
    cv::waitKey(10);
}


/* ==================== Private Functions ====================*/

void ImageHandler::resizeImage(const cv::Mat& cv_img) {
    /* ---  cv img crop, using the center of height --- */
    int cv_img_width = cv_img.cols;
    int cv_img_height = cv_img.rows;
    int cv_expect_height = cv_img_width/16*9;
    if (cv_img_height > cv_expect_height)
    {
        cv::Rect myROI(0, (cv_img_height-cv_expect_height)/2, cv_img_width, cv_expect_height);
        _cv_img = cv_img(myROI);
    }
    _cv_img = cv_img;
}


void ImageHandler::detect() {
    auto bboxt_lists = _detector.detect(_cv_img, ImageHandler::_detector_threshold);
    _is_detected = bboxt_lists.size() > 0;
    if (_is_detected)
    { 
        _detect_box.x = bboxt_lists[0].x;
        _detect_box.y = bboxt_lists[0].y;
        _detect_box.width = bboxt_lists[0].w;
        _detect_box.height = bboxt_lists[0].h;
        _detect_prob = bboxt_lists[0].prob;
        ROS_INFO("Detected ====================================");
    }
}
/* FIXME: put into another thread

void ImageHandler::track_init(){
    if(_tracker != nullptr) {
        _tracker.release();
    }
    _tracker = cv::TrackerKCF::create(_tracker_params);
    _tracker->init(_cv_img, _detect_box);
}

void ImageHandler::track(){
    if(_tracker == nullptr) {
        return;
    }

    bool tracker_is_found = _tracker->update(_cv_img, _detect_box);
    if (tracker_is_found) 
    {
      _detect_prob -= 0.005;
    }
}
*/
void ImageHandler::drawBox(){
    if(_is_detected) {
        _is_detected = false;
        /* --- put text --- */
        cv::Rect roi(_detect_box.x, _detect_box.y, _detect_box.width, _detect_box.height);
        cv::rectangle(_cv_img, roi, cv::Scalar(0,255,0), 1, 8, 0 ); 
        putText(_cv_img, 
            "quad " + std::to_string(_detect_prob), 
            cv::Point2f(_detect_box.x, _detect_box.y - 14), 
            cv::FONT_HERSHEY_COMPLEX_SMALL, 
            1.2, 
            cv::Scalar(255, 255, 0), 
            2
        ); 
    }
}

