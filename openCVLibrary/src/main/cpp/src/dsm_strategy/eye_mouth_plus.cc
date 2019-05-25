//
// Created by untouch on 18-12-17.
//

#include "eye_mouth_plus.h"
#include "stats_basic.h"
#include <cmath>
#include <mutex>

void EyeMouthStatusPlus::Feed(const std::vector<cv::Point2f> &landmarks) {
    landmarks_ = landmarks;
    InitMaxEyeRatio();
    GetEyeSize(landmarks_, leftSize_, rightSize_);
    if(!initMaxEyeSize){
        InitMaxEyeSize(leftSize_, rightSize_);
    }
}

bool EyeMouthStatusPlus::GetEyeStatus() {
    auto left = GetLeftEyeRatio();
    auto right = GetRightEyeRatio();
    return left < left_eye_threshold_ and right < right_eye_threshold_;
}

bool EyeMouthStatusPlus::GetMouthStatus() {
    auto mouth = GetMouthRatio();

    std::cout << "mouth : " << mouth << " threshold : " << mouth_threshold_ << std::endl;
    return mouth > mouth_threshold_;
}

float EyeMouthStatusPlus::GetLeftEyeRatio() {
    float dist0 = norm(landmarks_[44], landmarks_[48]);
    float dist1 = norm(landmarks_[45], landmarks_[51]);
    float dist2 = norm(landmarks_[46], landmarks_[50]);
    float dist3 = norm(landmarks_[47], landmarks_[49]);
    return (dist1 + dist2 + dist3) / dist0;
}

float EyeMouthStatusPlus::GetRightEyeRatio() {
    float dist0 = norm(landmarks_[52], landmarks_[56]);
    float dist1 = norm(landmarks_[53], landmarks_[59]);
    float dist2 = norm(landmarks_[54], landmarks_[58]);
    float dist3 = norm(landmarks_[55], landmarks_[57]);
    return (dist1 + dist2 + dist3) / dist0;
}

#include <algorithm>
float EyeMouthStatusPlus::GetMouthRatio() {
    float dist0 = norm(landmarks_[72], landmarks_[76]);
    float dist1 = norm(landmarks_[73], landmarks_[79]);
    float dist2 = norm(landmarks_[74], landmarks_[78]);
    float dist3 = norm(landmarks_[75], landmarks_[77]);
    auto val = (dist1 + dist2 + dist3) / dist0;
    if (window_.size() == size_) window_.pop_front();

    window_.push_back(val);
    auto sum = std::accumulate(window_.begin(), window_.end(),0.f);
    return sum/ static_cast<float>(window_.size());
}

bool EyeMouthStatusPlus::Calibrate() {
    sort(right_eye_ratios_.begin(), right_eye_ratios_.end());
    sort(left_eye_ratios_.begin(), left_eye_ratios_.end());
    sort(mouth_ratios_.begin(), mouth_ratios_.end());

    size_t right_size = right_eye_ratios_.size()/2;
    size_t left_size = left_eye_ratios_.size()/2;
    size_t mouth_size = mouth_ratios_.size()/2;

    for(auto iter = right_eye_ratios_.begin(); iter != right_eye_ratios_.begin()+right_size; ++iter)
        *(iter + right_size) = *iter * 0.5f;
    for(auto iter = left_eye_ratios_.begin(); iter != left_eye_ratios_.begin()+left_size; ++iter)
        *(iter + left_size) = *iter * 0.5f;
    for(auto iter = mouth_ratios_.begin(); iter != mouth_ratios_.begin()+mouth_size; ++iter)
        *(iter + left_size) = *iter * 10.f;

    cv::Mat train_data_left(left_eye_ratios_, true);
    cv::Mat train_data_right(right_eye_ratios_, true);
    cv::Mat train_labels_left(100, 1, CV_32SC1);
    cv::Mat train_labels_right(100, 1, CV_32SC1);

    cv::Mat centers_left(2, 1, train_data_left.type());
    cv::Mat centers_right(2, 1, train_data_right.type());

    kmeans(train_data_left, 2, train_labels_left,
           cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),
           3, cv::KMEANS_PP_CENTERS, centers_left);
    kmeans(train_data_right, 2, train_labels_right,
           cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),
           3, cv::KMEANS_PP_CENTERS, centers_right);

    left_eye_threshold_ = (centers_left.at<float>(0) + centers_left.at<float>(1)) / 2;
    right_eye_threshold_ = (centers_right.at<float>(0) + centers_right.at<float>(1)) / 2;
    if (centers_right.at<float>(0) > centers_right.at<float>(1)) {
        right_eye_thresh_open = centers_right.at<float>(0);
        right_eye_thresh_close = centers_right.at<float>(1);
    } else {
        right_eye_thresh_open = centers_right.at<float>(1);
        right_eye_thresh_close = centers_right.at<float>(0);
    }

    if (centers_left.at<float>(0) > centers_left.at<float>(1)) {
        left_eye_thresh_open = centers_left.at<float>(0);
        left_eye_thresh_close = centers_left.at<float>(1);
    } else {
        left_eye_thresh_open = centers_left.at<float>(1);
        left_eye_thresh_close = centers_left.at<float>(0);
    }
    mouth_threshold_ = computeMean(mouth_ratios_.begin() + mouth_size, mouth_ratios_.end());

    calibration_flag_ = false;
    return true;
}

void EyeMouthStatusPlus::GetEyeSize(std::vector<cv::Point2f> &ld, float &left, float &right) {
    if(ld.empty())
        return;
    auto Norm = [&](cv::Point2f p1, cv::Point2f p2){
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                    (p1.y - p2.y) * (p1.y - p2.y));
    };


    float distl1 = Norm(ld[46], ld[50]);
    float distl2 = Norm(ld[47], ld[49]);
    float distr1 = Norm(ld[54], ld[58]);
    float distr2 = Norm(ld[55], ld[57]);

    left = (distl1 + distl2)/2;
    right = (distr1 + distr2)/2;
}

void EyeMouthStatusPlus::InitMaxEyeSize(float left, float right) {
    static int count = 20;
    if (--count > 0) {
        maxLeftSize_ = maxLeftSize_ > left ? maxLeftSize_ : left;
        maxRightSize_ = maxRightSize_ > right ? maxRightSize_ : right;
    } else initMaxEyeSize = true;
}

void EyeMouthStatusPlus::InitMaxEyeRatio() {
    auto left = GetLeftEyeRatio();
    auto right = GetRightEyeRatio();

    if (maxLeftRatio and left - maxLeftRatio > 0.3)
        left = maxLeftRatio;
    if (maxRighRatio and right - maxRighRatio > 0.3)
        right = maxRighRatio;

    maxLeftRatio = maxLeftRatio > left ? maxLeftRatio : left;
    maxRighRatio = maxRighRatio > right ? maxRighRatio : right;

    if (maxLeftRatio - left > 0.2) {
        static int count = 0;
        if (++count > 50) {
            maxLeftRatio -= 0.2;
            count = 0;
        }
    }
    if (maxRighRatio - left > 0.2) {
        static int count = 0;
        if (++count > 50) {
            maxRighRatio -= 0.2;
            count = 0;
        }
    }
//    std::cout << "maxLeft : " << maxLeftRatio << "  " << "left : " << left
//              << "maxRight : " << maxRighRatio << "  " << "right : " << right
//              << std::endl;
}

void EyeMouthStatusPlus::ClearEyeRatio() {
    maxLeftRatio = 0.f;
    maxRighRatio = 0.f;
}

bool EyeMouthStatusPlus::SetLeftEyeThreshold(float threshold) {
    left_eye_threshold_ = threshold;
    return true;
}

bool EyeMouthStatusPlus::SetRightEyeThreshold(float threshold) {
    right_eye_threshold_ = threshold;
    return true;
}

bool EyeMouthStatusPlus::SetOpenMouthThreshold(float threshold) {
    mouth_threshold_ = threshold;
    return true;
}

EyeMouthStatusPlus::EyeMouthResult EyeMouthStatusPlus::GetEyeMouthResult() {
    EyeMouthResult result;
    result.have_left_eye_ = true;
    result.have_right_eye_ = true;
    result.left_eye_open_rate_ = GetLeftEyeRatio();
    result.right_eye_open_rate_ = GetRightEyeRatio();
    result.left_eye_open_ = result.left_eye_open_rate_ > left_eye_threshold_;
    result.right_eye_open_ = result.right_eye_open_rate_ > right_eye_threshold_;

    //mouth infomation
    result.have_mouth_ = true;
    auto mouth_rate = GetMouthRatio();
    result.mouth_open_rate_ = (1 / (1 + std::exp(0.f - mouth_rate))) * 2.f - 1.f;
    result.mouth_open_ = mouth_rate > mouth_threshold_;

    return std::move(result);
}
