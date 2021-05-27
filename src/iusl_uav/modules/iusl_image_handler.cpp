#include <iusl_uav/iusl_image_handler.h>

using namespace iusl_uav;

ImageHandler::ImageHandler():
    _cv_img(),
    _cv_window_name("view"),
    _is_detected(false),
    _image_detector(new ImageDetector)
{
    cv::namedWindow(_cv_window_name);
    ROS_INFO("Image Handler init!");
}

ImageHandler::~ImageHandler() {
    cv::destroyWindow(_cv_window_name);
}


void ImageHandler::updateImage(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat cv_img = cv_bridge::toCvShare(msg, "bgr8")->image; 
    if (cv_img.empty()) return; 

    resizeImage(cv_img);
}

void ImageHandler::detectAndTrack() {
    _is_detected = _image_detector->detect(_cv_img);
}


void ImageHandler::imageDisplay()
{
    if (_cv_img.empty()) return;
    
    drawBox();

    cv::imshow("view", _cv_img);
    cv::waitKey(10);
}


const DetectionBoxInfo& ImageHandler::getDetectBoxInfo () const{
 return _image_detector->getDetectBoxInfo();
}

const int& ImageHandler::getNetWidth() const {
    return _image_detector->getNetWidth();
}

const int& ImageHandler::getNetHeight() const {
    return _image_detector->getNetHeight();
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
void ImageHandler::drawBox() {
    if(_is_detected) {
        _is_detected = false;
        cv::Rect2d detect_box = _image_detector->getDetectBox();
        /* --- put text --- */
        cv::Rect roi(detect_box.x, detect_box.y, detect_box.width, detect_box.height);
        cv::rectangle(_cv_img, roi, cv::Scalar(0,255,0), 1, 8, 0 ); 
        putText(_cv_img, 
            "quad " + std::to_string(_image_detector->getDetectProb()), 
            cv::Point2f(detect_box.x, detect_box.y - 14), 
            cv::FONT_HERSHEY_COMPLEX_SMALL, 
            1.2, 
            cv::Scalar(255, 255, 0), 
            2
        ); 
    }
}
