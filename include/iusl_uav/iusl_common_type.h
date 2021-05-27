#ifndef _IUSL_COMMON_TYPE_
#define _IUSL_COMMON_TYPE_

namespace iusl_uav {
    struct VehicleStatus {
        float gim_pitch_now;
        float gim_yaw_now;
        float gim_roll_now;
        float UAV_lat_now;
        float UAV_lon_now;
        float UAV_alt_now;
        float UAV_vx_now;
        float UAV_vy_now;
        float laser_dis_now;
        uint8_t flight_state; 

        VehicleStatus() {
            gim_pitch_now = 0.;
            gim_yaw_now = 0.;
            gim_roll_now = 0.;
            UAV_lat_now = 0.;
            UAV_lon_now = 0.;
            UAV_alt_now = 0.;
            UAV_vx_now = 0.;
            UAV_vy_now = 0.;
            laser_dis_now = 0.;
            flight_state = 0; 
        }
    };

    struct DetectionBoxInfo {
        int64_t center_x;
        int64_t center_y;
        int64_t max_length;

        DetectionBoxInfo() {
            center_x = 0.;
            center_y = 0.;
            max_length = 0.;
        }

    };
}

#endif