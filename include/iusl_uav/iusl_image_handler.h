#ifndef _IUSL_IMAGE_HANDLER_
#define _IUSL_IMAGE_HANDLER_

#define IMAGE_HANDLER_DEBUG 1

#include <string>
#include <cv_bridge/cv_bridge.h>
#include <iusl_uav/iusl_image_detector.h>
#include <ros/ros.h>


namespace iusl_uav {
        
    class ImageHandler {  

        using uniptr_image_detector = std::unique_ptr<ImageDetector>; 

        public:
            ImageHandler();
            ~ImageHandler();

            void updateImage(const sensor_msgs::ImageConstPtr& msg);
            void imageDisplay();
            // TODO: remove track
            void detectAndTrack();

            const int& getNetWidth() const;
            const int& getNetHeight() const;

            const DetectionBoxInfo& getDetectBoxInfo() const;

        private:            
            cv::Mat _cv_img;
            std::string _cv_window_name;
            bool _is_detected;

            uniptr_image_detector _image_detector;


            void resizeImage(const cv::Mat&);
            void drawBox();
            
    };

#endif   
}