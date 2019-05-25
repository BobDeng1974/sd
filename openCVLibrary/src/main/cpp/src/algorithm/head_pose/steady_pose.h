//
// Created by slam on 19-4-30.
//

#ifndef FACE_DETECTION_FAST_RCNN_STEADY_POSE_H
#define FACE_DETECTION_FAST_RCNN_STEADY_POSE_H

#include <iostream>
#include <opencv2/opencv.hpp>

class SteadyPose{
public:
//    cv::KalmanFilter
    SteadyPose(int state_num, int measure_num, double cov_process, double cov_measure);
    SteadyPose(const SteadyPose&);
    void Init();
    void update(cv::Mat& measurement);
    float get_state();

private:
    int state_num_;
    int measure_num_;
    double cov_process_;
    double cov_measure_;
    cv::KalmanFilter filter_;
    cv::Mat state_;
    cv::Mat measurement_;
    cv::Mat prediction_;
};

#endif //FACE_DETECTION_FAST_RCNN_STEADY_POSE_H
