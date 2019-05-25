//
// Created by slam on 19-5-5.
//

#include "steady_pose.h"

SteadyPose::SteadyPose(int state_num, int measure_num, double cov_process, double cov_measure) :
                        state_num_(state_num),
                        measure_num_(measure_num),
                        cov_process_(cov_process),
                        cov_measure_(cov_measure){
    assert(this->state_num_==4 || this->state_num_==2);
    this->filter_ = cv::KalmanFilter(this->state_num_, this->measure_num_, 0, 6);
/*    this->state_ = cv::Mat(this->state_num_, 1, CV_64F);
    cv::randn(this->state_, cv::Scalar::all(0), cv::Scalar::all(0.1));*/
    this->state_ = cv::Mat::zeros(this->state_num_, 1, CV_64F);
    this->measurement_ = cv::Mat::zeros(this->measure_num_, 1, CV_64F);
    this->prediction_ = cv::Mat::zeros(this->state_num_, 1, CV_64F);
    if (this->measure_num_==1){
        Init();
    }
    if(this->measure_num_==2){
        this->filter_.transitionMatrix = (cv::Mat_<double>(4, 4) << 1.f, 0.f, 1.f, 0.f,
                                                                    0.f, 1.f, 0.f, 1.f,
                                                                    0.f, 0.f, 1.f, 0.f,
                                                                    0.f, 0.f, 0.f, 1.f);
        this->filter_.measurementMatrix = (cv::Mat_<double>(2, 4) << 1.f, 0.f, 0.f, 0.f,
                                                                    0.f, 1.f, 0.f, 0.f);
        this->filter_.processNoiseCov = (cv::Mat_<double>(4, 4) << 1.f, 0.f, 0.f, 0.f,
                                                                    0.f, 1.f, 0.f, 0.f,
                                                                    0.f, 0.f, 1.f, 0.f,
                                                                    0.f, 0.f, 0.f, 1.f)*this->cov_process_;
        this->filter_.measurementNoiseCov = (cv::Mat_<double>(2, 2)<< 1.f, 0.f,
                                                                        0.f, 1.f)* this->cov_measure_;
    }
/*    cv::setIdentity(this->filter_.measurementMatrix);
    cv::setIdentity(this->filter_.processNoiseCov, cv::Scalar::all(this->cov_process_));
    cv::setIdentity(this->filter_.measurementNoiseCov, cv::Scalar::all(this->cov_measure_));
    cv::setIdentity(this->filter_.errorCovPost, cv::Scalar::all(1));
    cv::randn(this->filter_.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));*/
}

SteadyPose::SteadyPose(const SteadyPose& sp):
                        state_num_(sp.state_num_),
                        measure_num_(sp.measure_num_),
                        cov_process_(sp.cov_process_),
                        cov_measure_(sp.cov_measure_)
                        {
    this->filter_ = sp.filter_;
    this->state_ = sp.state_.clone();
    this->measurement_ = sp.measurement_.clone();
    this->prediction_ = sp.prediction_.clone();
}

void SteadyPose::update(cv::Mat& measurement) {
    this->prediction_ = this->filter_.predict();
    this->measurement_ = measurement;
    this->filter_.correct(this->measurement_);
    this->state_ = this->filter_.statePost;
}

float SteadyPose::get_state(){
    return this->state_.at<double>(0);
}

void SteadyPose::Init() {
    this->filter_.transitionMatrix = (cv::Mat_<double>(2, 2) << 1.f, 1.f,
            0.f, 1.f);
    this->filter_.measurementMatrix = (cv::Mat_<double>(1, 2) << 1.f, 1.f);
    this->filter_.processNoiseCov = (cv::Mat_<double>(2, 2) << 1.f, 0.f,
            0.f, 1.f)*this->cov_process_;
    this->filter_.measurementNoiseCov = (cv::Mat_<double>(1, 1)<<1.f*this->cov_measure_);
}
