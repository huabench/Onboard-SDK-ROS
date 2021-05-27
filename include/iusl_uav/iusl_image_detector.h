#ifndef _IUSL_IMAGE_DETECTOR_
#define _IUSL_IMAGE_DETECTOR_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <iusl_uav/yolo_v2_class.hpp>
#include <iusl_uav/iusl_common_type.h>

namespace iusl_uav {
    enum class DetectorState {
        NOTHING,
        DETECTED,
        TRACKING
    };

    class ImageDetector {
        public:
            ImageDetector();
            ~ImageDetector();

            bool detect(const cv::Mat&);

            const int& getNetWidth() const;
            const int& getNetHeight() const;
            const cv::Rect2d& getDetectBox() const;
            const float& getDetectProb() const;

            const DetectionBoxInfo& getDetectBoxInfo() const;

        private:
            std::string _pkg_path;
            std::string _cv_data_file;
            std::string _cv_cfg_file;
            std::string _cv_weight_file;

            Detector _detector;
            cv::Ptr<cv::Tracker> _tracker;
            cv::TrackerKCF::Params _tracker_params;
            cv::Rect2d _detect_box;
            
            float _detect_prob;
            DetectorState _detector_state; // TODO: may be not neccessary
            const float _detector_threshold;
            DetectionBoxInfo _detect_box_info;


            const int _net_width;
            const int _net_height;

    };
}
#endif