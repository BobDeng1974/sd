#pragma once

#include <memory>
#include <string>
#include <opencv2/opencv.hpp>

#include "tvm_runner.h"

class ShapePredictor {
public:
//    ShapePredictor(std::string module_path,std::string json_path,std::string params_path);
    ShapePredictor(const std::string& path);
    std::vector<cv::Point2f> predict(const cv::Mat& face,cv::Rect& bbox);
private:
    std::shared_ptr<TvmRunner> shape_predictor_p;
    std::shared_ptr<TvmData> in,out;
};