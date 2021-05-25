#ifndef _IUSL_IMAGE_HANDLER_
#define _IUSL_IMAGE_HANDLER_

#define IMAGE_HANDLER_DEBUG 1

#include <cv_bridge/cv_bridge.h> 
#include <ros/package.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <string>
#include <iusl_uav/yolo_v2_class.hpp>
#ifdef IMAGE_HANDLER_DEBUG
#include<ros/ros.h>
#endif

namespace iusl_image_handler {
    
    enum class DetectorState {
        NOTHING,
        DETECTED,
        TRACKING
    };

    class ImageHandler {

        public:
            ImageHandler();
            ~ImageHandler();

            void updateImage(const sensor_msgs::ImageConstPtr& msg);
            void imageDisplay();
            // TODO: remove track
            void detectAndTrack();

        private:            
            cv::Mat _cv_img;
            std::string _cv_window_name;
            std::string pkg_path;
            std::string _cv_data_file;
            std::string _cv_cfg_file;
            std::string _cv_weight_file;


            Detector _detector;
            cv::Ptr<cv::Tracker> _tracker;
            cv::TrackerKCF::Params _tracker_params;
            cv::Rect2d _detect_box;
            float _detect_prob;
            bool _is_detected = false;
            DetectorState _detector_state; // TODO: may be not neccessary

            const float _detector_threshold;

            void resizeImage(const cv::Mat&);

            void detect();

            //void track();
            //void track_init();

            void drawBox();

    };

#endif   
}