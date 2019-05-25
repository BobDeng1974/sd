//
// Created by slam on 18-10-17.
//
#include "shape_predict.h"

ShapePredictor::ShapePredictor(const std::string &path_root):
    net_(new ncnn::Net){
    std::string param = path_root + "/landmark.param";
    std::string model = path_root + "/landmark.bin";

    net_->load_param(param.c_str());
    net_->load_model(model.c_str());
}


cv::Rect EnlargeBBox(cv::Rect bbox,int img_width,int img_height) {
    int x, y, width, height, x_c, y_c;
    x = bbox.x; y = bbox.y; width = bbox.width; height = bbox.height;
    x_c = x + width/2; y_c = y + height/2;
    int x_new, y_new, w_new, h_new;
    double scale = 1.1;
    w_new = static_cast<int>(scale * width);
    h_new = static_cast<int>(scale * height);
    x_new = std::max(x_c - w_new/2, 0);
    y_new = std::max(y_c - h_new/2, 0);
    w_new = (x_new + w_new) < img_width ? w_new : (img_width - x_new);
    h_new = (y_new + h_new) < img_height ? h_new : (img_height - y_new);
    cv::Rect face_bbox_adjust = cv::Rect(x_new, y_new, w_new, h_new);
    return face_bbox_adjust;
}


std::vector<cv::Point2f> ShapePredictor::predict(const cv::Mat &img, const cv::Rect &face) {
    std::vector<cv::Point2f> shape;
    // ZH : no need to clone one , just use img to crop a rect.
    cv::Rect face_bbox_adjust = EnlargeBBox(face,img.cols,img.rows);
    cv::Mat image = img.clone();
    cv::Mat face_img = image(face_bbox_adjust);
    cv::cvtColor(face_img, face_img, cv::COLOR_BGR2GRAY);

    // ZH : no need to draw rectangle ...
    cv::rectangle(image,face, cv::Scalar(0,255,0));

    ncnn::Mat in = ncnn::Mat::from_pixels_resize(face_img.data, ncnn::Mat::PIXEL_GRAY, face_img.cols, face_img.rows, 112, 112);
    ncnn::Mat out;

    ncnn::Extractor ex = net_->create_extractor();
    // ZH : what happen if I set using 4 thread here ?
    ex.set_num_threads(1);
    ex.input("data", in);
    ex.extract("pts82",out);

    for(size_t i = 0; i != out.total(); i+=2){
        float x = out[i];
        float y = out[i+1];
        x = x/2 + meanshape[i];
        y = y/2 + meanshape[i+1];
        x = x * face_bbox_adjust.width + face_bbox_adjust.x;
        y = y * face_bbox_adjust.height + face_bbox_adjust.y;
        standard_face_[i/2].x = meanshape[i] * face_bbox_adjust.width + face_bbox_adjust.x;
        standard_face_[i/2].y = meanshape[i+1]* face_bbox_adjust.height + face_bbox_adjust.y;
        shape.emplace_back(x,y);
    }
    return shape;
}

