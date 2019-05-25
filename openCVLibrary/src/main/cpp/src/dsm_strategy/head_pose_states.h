//
// Created by untouch on 19-1-16.
//

#ifndef DSM_CPP_HEAD_POSE_STATES_H
#define DSM_CPP_HEAD_POSE_STATES_H


class HeadPoseStatus{
public:
    HeadPoseStatus():calibration_(false),
                     vert_upper_thres_(0.f), vert_bottom_thres_(0.f),
                     hori_left_thres_(0.f),hori_right_thres_(0.f){}
    bool SetParam(float left_diff, float right_diff, float up_diff, float down_diff);
    bool Calibration(float yaw, float pitch);
    int GetHeadStatus(float pitch, float yaw); //0 正常, 1234 : 上下左右

private:
    bool calibration_;
    float up_diff_;
    float down_diff_;

    float left_diff_;
    float right_diff_;
    //垂直方向的两个临界值
    float vert_upper_thres_;        ///< 垂直方向上的临界值
    float vert_bottom_thres_;       ///< 垂直方向下的临界值

    //水平方向的两个临界值
    float hori_left_thres_;        ///< 水平方向左侧的临界值
    float hori_right_thres_;       ///< 水平方向右侧的临界值
};


#endif //DSM_CPP_HEAD_POSE_STATES_H
