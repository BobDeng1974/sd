//
// Created by amanda on 18-10-17.
//
#include <opencv2/opencv.hpp>
#include "faceid_ncnn.h"

FaceIDNCNN::FaceIDNCNN(const std::string &model_folder_path) {
    std::string model_path = model_folder_path+"/wd_face_reg_v4.bin";
    std::string proto_path = model_folder_path+"/wd_face_reg_v4.param";
    face_net.load_param(proto_path.c_str());
    face_net.load_model(model_path.c_str());
}

void FaceIDNCNN::GetFaceFeature(const cv::Mat &align, cv::Mat &feat) {
    if(align.empty()) {
        return;
    }

    cv::Mat input_blob = cv::dnn::blobFromImage( align, 1.,
                                                 cv::Size(96,96), cv::Scalar(104,117,123), 0 );//for mobilefacenet_96

    ncnn::Mat in = ncnn::Mat::from_pixels(align.data, ncnn::Mat::PIXEL_BGR, align.cols, align.rows);
    ncnn::Mat out;

    ncnn::Extractor ex1 = face_net.create_extractor();
    ex1.set_num_threads(1);

    ex1.input("data",in);
    ex1.extract("feat",out);
    float* data = (float*)out.data;
    feat = cv::Mat::zeros(1,out.total(),CV_32FC1);
    for(size_t i=0;i < out.total();i++)
        feat.at<float>(0,i) = *(data+i);
}

float FaceIDNCNN::CalcCosScore(const cv::Mat& lr, const cv::Mat& rr)  {
    cv::Mat mult = lr * rr.t();
    double score = mult.at<float>(0,0)/ ( cv::norm( lr, cv::NORM_L2 ) * cv::norm(rr,cv::NORM_L2) );
    if(score < 0)
        score = 0.f;
    return score;
}
