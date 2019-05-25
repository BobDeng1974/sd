//
// Created by untouch on 18-12-17.
//

#ifndef DSM_CPP_EYE_MOUTH_PLUS_H
#define DSM_CPP_EYE_MOUTH_PLUS_H

#include <iostream>
#include <opencv2/opencv.hpp>

class EyeMouthStatusPlus{
public:
    struct EyeMouthResult{
        //eye infomation
        bool have_left_eye_;
        bool have_right_eye_;
        bool left_eye_open_;
        bool right_eye_open_;
        float left_eye_open_rate_;
        float right_eye_open_rate_;

        //mouth infomation
        bool have_mouth_;
        bool mouth_open_;
        float mouth_open_rate_;

        EyeMouthResult()
            : have_left_eye_(true),
              have_right_eye_(true),
              left_eye_open_(true),
              right_eye_open_(true),
              left_eye_open_rate_(0.f),
              right_eye_open_rate_(0.f),
              have_mouth_(true),
              mouth_open_(false),
              mouth_open_rate_(0.f){}
    };

    explicit EyeMouthStatusPlus(float eye_threshold = 0.45f, float mouth_threshold = 2.2f)
        : left_eye_threshold_(eye_threshold),
          right_eye_threshold_(eye_threshold),
          mouth_threshold_(mouth_threshold),
          calibration_flag_(true),
          size_(2){};

    ~EyeMouthStatusPlus() = default;
    void Feed(const std::vector<cv::Point2f>& landmarks);
    EyeMouthResult GetEyeMouthResult();

    bool SetLeftEyeThreshold(float threshold);
    bool SetRightEyeThreshold(float threshold);
    bool SetOpenMouthThreshold(float threshold);

    void ClearEyeRatio();
private:
    bool GetEyeStatus();
    bool GetMouthStatus();
    bool Calibrate();
    float GetLeftEyeRatio();
    float GetRightEyeRatio();
    float GetMouthRatio();

    void GetEyeSize(std::vector<cv::Point2f> &ld, float &left, float &right);
    void InitMaxEyeSize(float left, float right);

    void InitMaxEyeRatio();
private:
    float left_eye_thresh_open,right_eye_thresh_open;
    float left_eye_thresh_close,right_eye_thresh_close;
    float left_eye_threshold_;
    float right_eye_threshold_;
    float mouth_threshold_;
    bool calibration_flag_;

    std::vector<float> right_eye_ratios_;
    std::vector<float> left_eye_ratios_;
    std::vector<float> mouth_ratios_;
    std::vector<cv::Point2f> landmarks_;

    float leftSize_ = 0.f;
    float rightSize_= 0.f;
    float maxLeftSize_ = 0.f;
    float maxRightSize_ = 0.f;
    float maxLeftRatio = 0.f;
    float maxRighRatio = 0.f;
    bool initMaxEyeSize = false;

    std::deque<float> window_;
    int size_;
};


#endif //DSM_CPP_EYE_MOUTH_PLUS_H
