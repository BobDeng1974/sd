//
// Created by slam on 18-10-8.
//

//#include <bits/shared_ptr.h>
#include "head_pose_estimator.h"

HeadPoseEstimator::HeadPoseEstimator(cv::Mat &im) {
    double focal_length = im.cols; // Approximate focal length.
    cv::Point2f center = cv::Point2f(im.cols / 2, im.rows / 2);
    camera_matrix_ = (cv::Mat_<double>(3, 3)
            << focal_length, 0., center.x, 0., focal_length, center.y, 0., 0., 1.);
    dist_coeffs_ = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

/*    model_points_.emplace_back(cv::Point3d(0.0f, 0.0f, 0.0f));               // Nose tip
    model_points_.emplace_back(cv::Point3d(0.0f, -330.0f, -65.0f));          // Chin
    model_points_.emplace_back(cv::Point3d(-225.0f, 170.0f, -135.0f));       // Left eye left corner
    model_points_.emplace_back(cv::Point3d(225.0f, 170.0f, -135.0f));        // Right eye right corner
    model_points_.emplace_back(cv::Point3d(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
    model_points_.emplace_back(cv::Point3d(150.0f, -150.0f, -125.0f));       // Right mouth corner*/
    model_points_.emplace_back(cv::Point3d(1.507832897781671306e+02, 1.276442484178730297e+02, 8.158095550537109375e+01));               // Nose tip
    model_points_.emplace_back(cv::Point3d(1.496863601457184245e+02, 1.725352861232383930e+02, 5.941432189941406250e+01));          // Chin
    model_points_.emplace_back(cv::Point3d(1.273787673722809473e+02, 1.084941231162875539e+02, 5.887000274658203125e+01));       // Left eye left corner
    model_points_.emplace_back(cv::Point3d(1.743798242378982764e+02, 1.096523678289675559e+02, 5.936354446411132812e+01));        // Right eye right corner
    model_points_.emplace_back(cv::Point3d(1.341587851203469768e+02, 1.445513287951899883e+02, 6.061173248291015625e+01));      // Left Mouth corner
    model_points_.emplace_back(cv::Point3d(1.651019962756586779e+02, 1.456323182588465386e+02, 6.150511932373046875e+01));       // Right mouth corner
}

HeadPoseEstimator::HeadPoseEstimator(const cv::Size &s) {
    double focal_length = s.width; // Approximate focal length.
    cv::Point2f center = cv::Point2f(s.width / 2, s.height / 2);
    camera_matrix_ = (cv::Mat_<double>(3, 3)
            << focal_length, 0., center.x, 0., focal_length, center.y, 0., 0., 1.);
    dist_coeffs_ = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

/*    model_points_.emplace_back(cv::Point3d(0.0f, 0.0f, 0.0f));               // Nose tip
    model_points_.emplace_back(cv::Point3d(0.0f, -330.0f, -65.0f));          // Chin
    model_points_.emplace_back(cv::Point3d(-225.0f, 170.0f, -135.0f));       // Left eye left corner
    model_points_.emplace_back(cv::Point3d(225.0f, 170.0f, -135.0f));        // Right eye right corner
    model_points_.emplace_back(cv::Point3d(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
    model_points_.emplace_back(cv::Point3d(150.0f, -150.0f, -125.0f));       // Right mouth corner*/
    model_points_.emplace_back(cv::Point3d(-1.157522897159338982e+00, -7.177086549646730873e+00, 3.069286441802978516e+01));               // Nose tip
    model_points_.emplace_back(cv::Point3d(-2.254452529608045097e+00, 3.771395115571863244e+01, 8.526230812072753906e+00));          // Chin
    model_points_.emplace_back(cv::Point3d(-2.456204530304552236e+01, -2.632721185123220664e+01, 7.981911659240722656e+00));       // Left eye left corner
    model_points_.emplace_back(cv::Point3d(2.243901156257180673e+01, -2.516896713855220469e+01, 8.475453376770019531e+00));        // Right eye right corner
    model_points_.emplace_back(cv::Point3d(-1.778202755497949283e+01, 9.729993827670227802e+00, 9.723641395568847656e+00));      // Left Mouth corner
    model_points_.emplace_back(cv::Point3d(1.316118360033220824e+01, 1.081098329132677804e+01, 1.061702823638916016e+01));       // Right mouth corner
}

HeadPoseEstimator::HeadPoseEstimator(std::string &model_path, const cv::Size &s){
    std::vector<cv::Point3d> model_points_3D;
    load_3D_facemodel_68(model_path, model_points_3D);

    std::vector<int> used_id_3D{36, 39, 42, 45,
                                27, 28, 29, 30, 31, 32, 33, 34, 35,
                                49, 50, 51, 52, 53, 61, 62, 63};
    for (auto & i : used_id_3D){
        model_points_.emplace_back(model_points_3D[i]);
    }

    double focal_length = s.width; // Approximate focal length.
    cv::Point2f center = cv::Point2f(s.width / 2, s.height / 2);
    camera_matrix_ = (cv::Mat_<double>(3, 3)
            << focal_length, 0., center.x, 0., focal_length, center.y, 0., 0., 1.);
    dist_coeffs_ = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

    this->rotation_vector_ = (cv::Mat_<double>(3, 1) << 0.01891013, 0.08560084, -3.14392813);
    this->translation_vector_ = (cv::Mat_<double>(3, 1) << -14.97821226, -10.62040383, -2053.03596872);

    std::shared_ptr<SteadyPose> stp0(new SteadyPose(2, 1, 0.1, 0.1));
    std::shared_ptr<SteadyPose> stp1(new SteadyPose(2, 1, 0.1, 0.1));
    std::shared_ptr<SteadyPose> stp2(new SteadyPose(2, 1, 0.1, 0.1));
    std::shared_ptr<SteadyPose> stp3(new SteadyPose(2, 1, 0.1, 0.1));
    std::shared_ptr<SteadyPose> stp4(new SteadyPose(2, 1, 0.1, 0.1));
    std::shared_ptr<SteadyPose> stp5(new SteadyPose(2, 1, 0.1, 0.1));
    stps_.emplace_back(stp0);
    stps_.emplace_back(stp1);
    stps_.emplace_back(stp2);
    stps_.emplace_back(stp3);
    stps_.emplace_back(stp4);
    stps_.emplace_back(stp5);
}

void HeadPoseEstimator::load_3D_facemodel_68(std::string &model_path, std::vector<cv::Point3d> &model_points_3D) {
    std::ifstream infile(model_path.c_str());
    std::string line;
    std::vector<float> temp;
    while(!infile.eof()){
        infile >> line;
        temp.emplace_back(atof(line.c_str()));
    }
    for (int i = 0; i < temp.size() / 3; i++) {
        model_points_3D.emplace_back(cv::Point3d(temp[i], temp[i + 68], -temp[i + 136]));
    }
}
cv::Vec3d HeadPoseEstimator::RotationMatrix2Euler(const cv::Matx33d &rotation_matrix) {
    double q0 = sqrt(1 + rotation_matrix(0, 0) + rotation_matrix(1, 1) + rotation_matrix(2, 2)) / 2.0;
    double q1 = (rotation_matrix(2, 1) - rotation_matrix(1, 2)) / (4.0 * q0);
    double q2 = (rotation_matrix(0, 2) - rotation_matrix(2, 0)) / (4.0 * q0);
    double q3 = (rotation_matrix(1, 0) - rotation_matrix(0, 1)) / (4.0 * q0);

    double t1 = 2.0 * (q0 * q2 + q1 * q3);

    double yaw = asin(2.0 * (q0 * q2 + q1 * q3));
    double pitch = atan2(2.0 * (q0 * q1 - q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    double roll = atan2(2.0 * (q0 * q3 - q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);

    return cv::Vec3d(pitch, yaw, roll);
}

cv::Vec3d HeadPoseEstimator::AxisAngle2Euler(const cv::Vec3d &axis_angle) {
    cv::Matx33d rotation_matrix;
    cv::Rodrigues(axis_angle, rotation_matrix);
    return RotationMatrix2Euler(rotation_matrix);
}

/*cv::Vec3d HeadPoseEstimator::estimate_pose(std::vector<cv::Point2f> image_points) {
//    std::vector<int> used_id_2D{44, 48, 52, 56, 38, 39, 40, 41, 42, 43, 63}; // 82 points
    std::vector<int> used_id_2D{44, 48, 52, 56,
                                35, 36, 37, 38, 39, 40, 41, 42, 43,
                                60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79}; // 82 points
//    std::vector<int> used_id_2D{36, 39, 42, 45, 30, 31, 32, 33, 34, 35, 51}; // 68 points
//    std::vector<int> used_id_2D{30, 8, 36, 45, 48, 54};
    std::vector<cv::Point2f> used_points_2D;
    for (auto & i : used_id_2D){
        used_points_2D.emplace_back(image_points[i]);
    }

    if(used_points_2D.size() != model_points_.size()){
        std::cout << "Numbers of 3D and 2D points are not matched!" << std::endl;
        exit(0);
    }

    // Output rotation and translation
//    cv::Mat rotation_vector; // Rotation in axis-angle form
//    cv::Mat translation_vector;

    // Solve for pose
    cv::solvePnP(model_points_, used_points_2D, camera_matrix_, dist_coeffs_, this->rotation_vector_, this->translation_vector_, true, cv::SOLVEPNP_EPNP);
    cv::Vec3d euler = AxisAngle2Euler(this->rotation_vector_);
    return euler;
}*/

cv::Vec3d HeadPoseEstimator::estimate_pose(const std::vector<cv::Point2f>& image_points) {
    std::vector<int> used_id_2D{44, 48, 52, 56,
                                35, 36, 37, 38, 39, 40, 41, 42, 43,
                                61, 62, 63, 64, 65, 73, 74, 75};
//    std::vector<int> used_id_2D{36, 39, 42, 45, 30, 31, 32, 33, 34, 35, 51}; // 68 points
//    std::vector<int> used_id_2D{38, 8, 44, 56, 60, 66};
    std::vector<cv::Point2f> used_points_2D;
    for (auto & i : used_id_2D){
        used_points_2D.emplace_back(image_points[i]);
    }

    if(used_points_2D.size() != model_points_.size()){
        std::cout << used_points_2D.size() << " " << model_points_.size() << std::endl;
        std::cout << "Numbers of 3D and 2D points are not matched!" << std::endl;
        exit(0);
    }

    // Solve for pose
    if(!this->rotation_vector_.empty()) {
        cv::solvePnP(model_points_, used_points_2D, camera_matrix_, dist_coeffs_, this->rotation_vector_,
                     this->translation_vector_, true);
    }
    else {
        cv::solvePnP(model_points_, used_points_2D, camera_matrix_, dist_coeffs_, this->rotation_vector_,
                     this->translation_vector_, false);
    }

    for (int i = 0; i < 3; i++) {
        cv::Mat tmp = this->rotation_vector_.rowRange(i, i+1);
        stps_[i]->update(tmp);
    }
    for (int i = 0; i < 3; i++) {
        cv::Mat tmp = this->translation_vector_.rowRange(i, i+1);
        stps_[i+3]->update(tmp);
    }
    float r1 = stps_[0]->get_state();
    float r2 = stps_[1]->get_state();
    float r3 = stps_[2]->get_state();
    float t1 = stps_[3]->get_state();
    float t2 = stps_[4]->get_state();
    float t3 = stps_[5]->get_state();
    this->rotation_vector_ = (cv::Mat_<double>(3, 1) << r1, r2, r3);
    this->translation_vector_ = (cv::Mat_<double>(3, 1) << t1, t2, t3);
    cv::Vec3d euler = AxisAngle2Euler(this->rotation_vector_);
    return euler;
}

cv::Vec4d HeadPoseEstimator::get_quaternion() {
    double r0 = this->rotation_vector_.at<double>(0,0);
    double r1 = this->rotation_vector_.at<double>(1,0);
    double r2 = this->rotation_vector_.at<double>(2,0);
    double theta = std::sqrt(r0*r0 + r1*r1 + r2*r2);
    double c = cos(theta*0.5);
    double s = sin(theta*0.5);
    double itheta = (theta!=0.) ? 1./theta : 0.;
    double w, q0, q1, q2;
    w = c;
    q0 = r0*itheta*s;
    q1 = r1*itheta*s;
    q2 = r2*itheta*s;
    cv::Vec4d Q_right(w, q0, q1, q2);
    cv::Vec4d Q_left = Q_right.conj();
    cv::Vec4d rot(0, 0, 0, 1);
    cv::Vec4d result = rot * Q_right;
    return result;
}

void HeadPoseEstimator::draw_pose(cv::Mat &image, const cv::Mat &rotation, const cv::Mat &translation,
                                  const cv::Scalar &color, int line_width) {
    int rear_size = 75, rear_depth = 0;
    int front_size = 100, front_depth = 100;
    std::vector<cv::Point3d> points_3d{cv::Point3d(-rear_size, -rear_size, rear_depth),
                                       cv::Point3d(-rear_size, rear_size, rear_depth),
                                       cv::Point3d(rear_size, rear_size, rear_depth),
                                       cv::Point3d(rear_size, -rear_size, rear_depth),
                                       cv::Point3d(-rear_size, -rear_size, rear_depth),
                                       cv::Point3d(-front_size, -front_size, front_depth),
                                       cv::Point3d(-front_size, front_size, front_depth),
                                       cv::Point3d(front_size, front_size, front_depth),
                                       cv::Point3d(front_size, -front_size, front_depth),
                                       cv::Point3d(-front_size, -front_size, front_depth),
                                       };
    std::vector<cv::Point2d> points_2d;
    std::vector<cv::Point2i> points_2i;
    cv::projectPoints(points_3d, rotation, translation, camera_matrix_, dist_coeffs_, points_2d);
    for (auto &pt : points_2d){
        points_2i.emplace_back(cv::Point2i(int(pt.x), int(pt.y)));
    }
    cv::polylines(image, points_2i, 1, color, line_width, cv::LINE_AA);
}

void HeadPoseEstimator::draw_pose(cv::Mat &image, const cv::Scalar &color, int line_width) {
    int rear_size = 75, rear_depth = 0;
    int front_size = 100, front_depth = 100;
    std::vector<cv::Point3d> points_3d{cv::Point3d(-rear_size, -rear_size, rear_depth),
                                       cv::Point3d(-rear_size, rear_size, rear_depth),
                                       cv::Point3d(rear_size, rear_size, rear_depth),
                                       cv::Point3d(rear_size, -rear_size, rear_depth),
                                       cv::Point3d(-rear_size, -rear_size, rear_depth),
                                       cv::Point3d(-front_size, -front_size, front_depth),
                                       cv::Point3d(-front_size, front_size, front_depth),
                                       cv::Point3d(front_size, front_size, front_depth),
                                       cv::Point3d(front_size, -front_size, front_depth),
                                       cv::Point3d(-front_size, -front_size, front_depth),
    };
    std::vector<cv::Point2d> points_2d;
    std::vector<cv::Point2i> points_2i;
    cv::projectPoints(points_3d, this->rotation_vector_, this->translation_vector_, camera_matrix_, dist_coeffs_, points_2d);
    for (auto &pt : points_2d){
        points_2i.emplace_back(cv::Point2i(int(pt.x), int(pt.y)));
    }
    cv::polylines(image, points_2i, 1, color, line_width, cv::LINE_AA);
}

void HeadPoseEstimator::reset() {
    for(const auto& steady_pose : stps_){
        steady_pose->Init();
    }
}
