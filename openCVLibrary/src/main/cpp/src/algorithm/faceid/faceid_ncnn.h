#pragma once
#include <string>
#include <opencv2/opencv.hpp>

#include "net.h"

class FaceIDNCNN {
public:
    void GetFaceFeature(const cv::Mat &align, cv::Mat &feat);
    static float CalcCosScore(const cv::Mat &lr, const cv::Mat &rr);

public:
    explicit FaceIDNCNN(const std::string& model_folder_path);

private:
    ncnn::Net face_net;

};