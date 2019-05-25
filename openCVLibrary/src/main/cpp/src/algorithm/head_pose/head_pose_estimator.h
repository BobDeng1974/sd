//
// Created by slam on 18-10-8.
//

#ifndef HEAD_POSE_ESTIMATOR_HEAD_POSE_EXTIMATOR_H
#define HEAD_POSE_ESTIMATOR_HEAD_POSE_EXTIMATOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <memory>
#include "steady_pose.h"

class HeadPoseEstimator {
public:
    HeadPoseEstimator(cv::Mat &im);
    HeadPoseEstimator(const cv::Size &s);
    HeadPoseEstimator(std::string &model_path, const cv::Size &s);
//    cv::Vec3d estimate_pose(std::vector<cv::Point2f> image_points);
    cv::Vec3d estimate_pose(const std::vector<cv::Point2f>& image_points);
    cv::Vec4d get_quaternion();
    void draw_pose(cv::Mat& image, const cv::Mat& rotation, const cv::Mat& translation, const cv::Scalar& color=cv::Scalar(255, 255, 255), int line_width=2);
    void draw_pose(cv::Mat& image, const cv::Scalar& color=cv::Scalar(255, 255, 255), int line_width=2);
    void reset();

private:
    void load_3D_facemodel_68(std::string &model_path, std::vector<cv::Point3d> &model_points_3D);
    cv::Vec3d RotationMatrix2Euler(const cv::Matx33d &rotation_matrix);
    cv::Vec3d AxisAngle2Euler(const cv::Vec3d &axis_angle);

    std::vector<cv::Point3d> model_points_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat rotation_vector_; // Rotation in axis-angle form
    cv::Mat translation_vector_;
    std::vector<std::shared_ptr<SteadyPose>> stps_;
};
#endif //HEAD_POSE_ESTIMATOR_HEAD_POSE_EXTIMATOR_H
