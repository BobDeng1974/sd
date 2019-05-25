//
// Created by untouch on 19-1-16.
//

#include <iostream>
#include "head_pose_states.h"

bool HeadPoseStatus::SetParam(float left_diff, float right_diff, float up_diff, float down_diff) {
    up_diff_ = up_diff;
    down_diff_ = down_diff;

    left_diff_ = left_diff;
    right_diff_ = right_diff;
}

int HeadPoseStatus::GetHeadStatus(float pitch, float yaw) {

//pitch 抬头角度过大，pitch不准。
    if(pitch > 50 and calibration_) return 3;
//    std::cout << "left : " << hori_left_thres_ << "  " << "right : " << hori_right_thres_
//              << "  " << "up : " << vert_upper_thres_ << "  " << "down : " << vert_bottom_thres_ << std::endl;

    int res = 0;
    if(yaw < hori_left_thres_) res = 1;
    else if(yaw > hori_right_thres_) res = 2;
    else if(pitch < vert_upper_thres_) res = 3;
    else if(pitch > vert_bottom_thres_) res = 4;

    if(calibration_) return res;
    return 0;

}

bool HeadPoseStatus::Calibration(float yaw, float pitch) {

    vert_upper_thres_ = pitch + up_diff_;
    vert_bottom_thres_ = pitch + down_diff_;
    hori_right_thres_ = yaw + right_diff_;
    hori_left_thres_ = yaw + left_diff_;

//    vert_upper_thres_ = up_diff_;
//    vert_bottom_thres_ = down_diff_;
//    hori_right_thres_ = right_diff_;
//    hori_left_thres_ = left_diff_;

    calibration_ = true;
    return true;
}


