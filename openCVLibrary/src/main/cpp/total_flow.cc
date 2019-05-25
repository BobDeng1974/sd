//
// Created by untouch.
//

#include "total_flow.h"
#include <cmath>
#include <pthread.h>
#include <sys/prctl.h>
#include <unistd.h>
#include "Util.h"
//#include <faceAttribute.h>

const int TotalFlow::FEATURE_LENGTH = 128;

const float TotalFlow:: same_face_thresh = 0.65;
const std::string TotalFlow::UnknowFace = std::string("UnknowFace");
const std::string TotalFlow::NoFace = std::string("NoFace");

TotalFlow::TotalFlow(const std::string& path) :
        assets_path_(path),
        config_path_(assets_path_ + "/configure.yaml"),
        sp_model_path_(assets_path_ + "/models/landmark"),
        head_pose_path_(assets_path_ + "/models/3D_model/model.txt"),
        faceid_path_(assets_path_ + "/models/faceid"),
        gaze_model_path_(assets_path_ + "/models/gaze_tracking"),
        object_model_path_(assets_path_ + "/models/object_detection"),
        align_method_(std::make_shared<FivePtsAlign>(96, 96)),
        face_align_(std::make_shared<FaceAlign>(align_method_.get())),
//        gaze_predict_(std::make_shared<GazePredict>(gaze_model_path_)),
        predictor_(std::make_shared<ShapePredictor>(sp_model_path_)),
        head_pose_estimator_(std::make_shared<HeadPoseEstimator>(head_pose_path_, cv::Size(640,480))),
        faceid_(std::make_shared<FaceIDNCNN>(faceid_path_)),
        object_detect_(std::make_shared<ObjectDetect>(object_model_path_)),
        config_(config_path_),
        eye_mouth_detector_(config_.eye_close_threshold_),
        first_time_flage_(true),
        keep_running_flag_(true),
        smoke_state_(false),
        call_state_(false),
        yaw_base_(0.f),
        pitch_base_(0.f),
        roll_base_(0.f),
        REGISTNUM(10),
        current_regist_num_(0),
        regist_over_flag_(false),
        mark_no_face_(false),
        face_match_flag_(true),
        face_match_count_(0){

    SmokeSetParam(config_.smoke_judge_time_, config_.smoke_judge_rate_/100.f);
    CallSetParam(config_.call_judge_time_, config_.call_judge_rate_/100.f);
    DozeSetParam(config_.doze_judge_time_, config_.call_judge_rate_/100.f);
    YawnSetParam(config_.yawn_judge_time_, config_.yawn_judge_rate_/100.f);
    DistractionSetParam(config_.distraction_judge_time_, config_.distraction_judge_rate_/100.f);

}


/**
 * 检测frame中的人脸信息
 * @param image
 * @return
 */
bool TotalFlow::DetectFrame(const cv::Mat &image) {
    auto obj_result = object_detect_->Detect(image);

    if(!obj_result->face()){
        std::lock_guard<std::mutex> lock_guard(landmark_mutex_);
        landmarks_.clear();
        face_bbox_ = cv::Rect();
        cerr << "[Error]:detect frame failed to find face" << endl;
        result_mutex_.lock();
        result_.set_have_face(false);
        result_mutex_.unlock();

        eye_mouth_detector_.ClearEyeRatio();
        mark_no_face_ = false;
        name_mutex_.lock();
        name_ = NoFace;
        name_mutex_.unlock();
        return false;
    }
    if(!mark_no_face_) {
        mark_no_face_ = true;
        head_pose_estimator_->reset();
    }

    face_bbox_ = obj_result->face_bbox();


    std::vector<cv::Point2f> shape = predictor_->predict(image, face_bbox_);
    std::vector<cv::Point2f> standard_face = predictor_->GetStandardFace();

    std::vector<cv::Point2f> l;
    std::vector<cv::Point2f> t;
//    std::vector<int> index{44, 48, 52, 56, 35, 36, 37, 38, 39, 40, 41, 42, 43, 61, 62, 63, 64, 65, 73, 74, 75};
//    std::vector<int> index{38, 8, 44, 56, 60, 66};
    std::vector<int> index{38, 8, 48, 52, 60, 66};
    for(auto idx : index){
        l.emplace_back(shape[idx]);
        t.emplace_back(standard_face[idx]);
    }

//    Mat H = findHomography(shape, standard_face, RANSAC );
    Mat H = findHomography(l, t, RANSAC );
    std::vector<cv::Point2f> convert_shape;
    for(const auto& point : shape){
        cv::Mat matrix = cv::Mat::ones(3, 1, CV_64FC1);
        matrix.at<double>(0,0) = point.x;
        matrix.at<double>(1,0) = point.y;
        cv::Mat res = H * matrix;

        auto num = res.at<double>(2,0);

        convert_shape.emplace_back(res.at<double>(0,0)/num, res.at<double>(1,0)/num);
    }
    eye_mouth_detector_.Feed(convert_shape);
    cv::Mat gray;
    if(config_.use_gaze_) {
        cv::cvtColor(image, gray, CV_BGR2GRAY);
//        gaze_predict_->Predict(gray, shape, gaze_direction_, HCSOrient_);
//        left_eye_center_ = gaze_predict_->GetLeftEyeCt();
//        right_eye_center_ = gaze_predict_->GetRightEyeCt();
    }

    std::unique_lock<std::mutex>uniqueLock(landmark_mutex_);
    landmarks_.clear();
    for(auto& point:shape){
        landmarks_.emplace_back(point);
    }
    uniqueLock.unlock();

    result_mutex_.lock();
    result_.set_have_face(true);
    PackageHelper::SetLandmarks(result_, landmarks_);
    result_mutex_.unlock();

    cv::Vec3d angles = head_pose_estimator_->estimate_pose(shape);
    quaternion_ = head_pose_estimator_->get_quaternion();

    std::unique_lock<std::mutex> uniqueLockAngles(angle_mutex_);
    angles_[0] = static_cast<float>(angles[0] * 180 / M_PI);
    angles_[1] = static_cast<float>(angles[1] * 180 / M_PI);
    angles_[2] = static_cast<float>(angles[2] * 180 / M_PI);
    angles_[2] = angles_[2] > 0 ? angles_[2] - 180 : angles_[2] + 180;

    uniqueLockAngles.unlock();

    call_state_ = obj_result->call();
    call_bbox_ = obj_result->call_bbox();
    smoke_state_ = obj_result->smoke();
    smoke_bbox_ = obj_result->smoke_bbox();

    std::vector<cv::Point2f> mouth_vec;
    for (size_t index = 60; index != 73; ++index) {
        mouth_vec.push_back(shape[index]);
    }
    auto mouth_bbox = cv::boundingRect(mouth_vec);

    if(mouth_bbox.area() > 10) {
        mouth_bbox.x = mouth_bbox.x - 0.5f * mouth_bbox.width;
        mouth_bbox.y = mouth_bbox.y - 0.5f * mouth_bbox.height;
        mouth_bbox.width = mouth_bbox.width * 2;
        mouth_bbox.height = mouth_bbox.height * 2;
        if(smoke_state_){
            auto i = smoke_bbox_ & mouth_bbox;
            float iou = static_cast<float>(i.area()) / (smoke_bbox_.area() + mouth_bbox.area() - i.area());
            if(iou == 0.f) smoke_state_ = false;
        }
    }


    name_mutex_.lock();
    if (name_ == TotalFlow::NoFace and angles_[1] > -15 and angles_[1] < 15) {
        face_match_flag_ = true;
        face_match_count_ = 0;
    } else if (name_ == TotalFlow::UnknowFace and angles_[1] > -15 and
               angles_[1] < 15 and face_match_count_++ < 10 /* config_.process_fps_*/) {
        face_match_flag_ = true;
    }

    name_mutex_.unlock();

    if(face_match_flag_){
        std::cout << "detect:" << face_match_count_ << std::endl;
        face_match_flag_ = false;
        std::vector<cv::Point2f> cv_pts;
        cv::Mat aligned_img;
        cv_pts.emplace_back(shape[44]);
        cv_pts.emplace_back(shape[56]);
        cv_pts.emplace_back(shape[38]);
        cv_pts.emplace_back(shape[60]);
        cv_pts.emplace_back(shape[66]);
        align_method_->set_im(image);
        align_method_->set_pts(cv_pts);
        aligned_img = face_align_->DoAlign();
        cv::Mat feat;
        faceid_->GetFaceFeature(aligned_img,feat);
        std::string name = GetName(feat);
        std::cout << "detect name:" << name << std::endl;
        name_mutex_.lock();
        name_ = name;
        name_mutex_.unlock();

        if (name != TotalFlow::UnknowFace and name != TotalFlow::NoFace)
            face_match_count_ = 0;
    }
    return true;
}


void CropAndResize(cv::Mat& src) {
    if (src.cols != 1280) {
        return;
    }
    int target_half_length = 960 / 2;
    int half_width = src.cols / 2;
    int left_x = half_width - target_half_length;
    cv::Rect bb(left_x,0,960,720);
    src = src(bb);
    cv::resize(src,src,cv::Size(640,480));
}





/**
 * 程序运行入口
 */
void TotalFlow::Run(cv::Mat& frame, Result& result, bool regist, const std::string& registName) {
    auto start = std::chrono::steady_clock::now();

    if(config_.save_image_) {
        auto time_stamp = TimeStamp::now().get_int64();
        auto write_path = assets_path_ + "/image/" + to_string(time_stamp) + ".png";
        cv::imwrite(write_path, frame);
    }

    frame_mutex_.lock();
    frame_ = frame.clone();
    CropAndResize(frame_);
    frame_mutex_.unlock();

    static std::once_flag s_once_flag;
    std::call_once(s_once_flag, [&](){
        LoadFeature();
        name_ = NoFace;
        process_image_thread_ = thread(mem_fn(&TotalFlow::ProcessImageThread), this);
        process_image_thread_.detach();

    });

    result_mutex_.lock();
    result = result_;
    result_mutex_.unlock();

    name_mutex_.lock();
    result.set_name(name_);
    result.set_camera_avaliable(true);
    name_mutex_.unlock();

    angle_mutex_.lock();
    cv::Point3f angles = angles_;
    angle_mutex_.unlock();
    angles.x -= pitch_base_;
    angles.y -= yaw_base_;
    angles.z -= roll_base_;
    PackageHelper::SetPoint3f(result.mutable_head_pose(), angles);

    bbox_mutex_.lock();
    PackageHelper::SetRect(result.mutable_face_bbox(),face_bbox_);
    bbox_mutex_.unlock();

    //添加unknownface*
    if(result.have_face() and result.name() == NoFace)
        result.set_name("UnknowFace*");

    auto end = std::chrono::steady_clock::now();
    auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    auto control_fps_cost = 1000 / config_.camera_fps_;
//    std::cout << "camera sleep : " << control_fps_cost-time_cost << std::endl;
    if(control_fps_cost > time_cost)
        std::this_thread::sleep_for(std::chrono::milliseconds(control_fps_cost-time_cost));
}

void TotalFlow::ProcessImageThread() {
    prctl(PR_SET_NAME, "DSM_Algorithm");
    while (true) {
        if (!keep_running_flag_) {
            break;
        }
        auto beg = std::chrono::steady_clock::now();
        RunProcess();
        auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now()-beg).count();
        auto control_fps_cost = 1000 / config_.process_fps_;
//        std::cout << "process sleep : " << control_fps_cost-time_cost << std::endl;
        if(control_fps_cost > time_cost)
            std::this_thread::sleep_for(std::chrono::milliseconds(control_fps_cost-time_cost));
    }
    std::cout << "keep_running_flag_ : " << keep_running_flag_ << std::endl;
    std::abort();
}


/**
 * 根据当前运行阶段,执行不同的程序
 */
void TotalFlow::RunProcess() {
    std::unique_lock<std::mutex> uniqueLock(frame_mutex_);
    cv::Mat frame =frame_.clone();
    uniqueLock.unlock();
    if (frame.empty()) {
        return;
    }
    bool success_flag = DetectFrame(frame);

    if(success_flag) RunMainStep();
    else {
        cv::Rect bbox;
        lock_guard<mutex>lock_guard(result_mutex_);
        PackageHelper::SetRect(result_.mutable_cigarette_bbox(), bbox);
        PackageHelper::SetRect(result_.mutable_phone_bbox(), bbox);

        result_.set_smoking(false);
        result_.set_phone(false);
        result_.set_doze(false);
        result_.set_yawn(false);
    }
}

/**
 * 程序主函数
 */
void TotalFlow::RunMainStep() {
    float pitch = angles_[0];
    float yaw = angles_[1];
    int head_forward = head_pose_detector_.GetHeadStatus(pitch, yaw);
    auto eye_mouth_result = eye_mouth_detector_.GetEyeMouthResult();    
    bool eye_state = !eye_mouth_result.right_eye_open_ && !eye_mouth_result.right_eye_open_;
    bool mouth_state = eye_mouth_result.mouth_open_;

    //如果角度超出一定范围，不做闭眼判断。
    if(yaw > 30 or yaw < -30 or pitch > 15 or pitch < -15) {
        eye_state = false;
    }

    auto smoke_value = smoke_judger_.Detect(smoke_state_);
    auto call_value = call_judger_.Detect(call_state_);
    auto doze_value = doze_judger_.Detect(eye_state);
    auto yawn_value = yawn_judger_.Detect(mouth_state);
    int distract_value = distract_judger_.Detect(static_cast<bool>(head_forward));

    cv::Rect bbox;
    lock_guard<std::mutex>lock_guard(result_mutex_);
    PackageHelper::SetRect(result_.mutable_cigarette_bbox(), smoke_bbox_);
    PackageHelper::SetRect(result_.mutable_phone_bbox(), call_bbox_);
    PackageHelper::SetQuaternion(result_.mutable_head_quaternion(), quaternion_);
    PackageHelper::SetPoint3f(result_.mutable_left_eye_center(), left_eye_center_);
    PackageHelper::SetPoint3f(result_.mutable_right_eye_center(), right_eye_center_);
    if(distract_value)
        PackageHelper::SetDistraction(result_, head_forward);

    result_.set_smoking(smoke_value);
    result_.set_phone(call_value);
    result_.set_doze(doze_value);
    result_.set_yawn(yawn_value);
    
    result_.set_mouth_open(eye_mouth_result.mouth_open_);
    result_.set_mouth_open_rate(eye_mouth_result.mouth_open_rate_);
    result_.set_have_mouth(eye_mouth_result.have_mouth_);
    
    result_.set_left_eye_open(eye_mouth_result.left_eye_open_);
    result_.set_right_eye_open(eye_mouth_result.right_eye_open_);
    result_.set_have_left_eye(eye_mouth_result.have_left_eye_);
    result_.set_have_right_eye(eye_mouth_result.have_right_eye_);
    result_.set_left_eye_open_rate(eye_mouth_result.left_eye_open_rate_);
    result_.set_right_eye_open_rate(eye_mouth_result.right_eye_open_rate_);

//    cv::Vec3f gaze_direction(0.f,0.f,0.f);
//    cv::Vec3f gaze_direction(static_cast<float>(HCSOrient_[0]),
//                             static_cast<float>(HCSOrient_[1]),
//                             static_cast<float>(HCSOrient_[2]));
    cv::Vec3f gaze_direction(static_cast<float>(gaze_direction_[0]),
                             static_cast<float>(gaze_direction_[1]),
                             static_cast<float>(gaze_direction_[2]));
    PackageHelper::SetPoint3f(result_.mutable_left_eye_direction(), gaze_direction);
    PackageHelper::SetPoint3f(result_.mutable_right_eye_direction(), gaze_direction);

}


bool TotalFlow::Regist(const std::string& feature_path, HEAD_FORWARD head_pose) {

    cv::Mat feature;

    frame_mutex_.lock();
    cv::Mat frame = frame_.clone();
    frame_mutex_.unlock();

    std::unique_lock<std::mutex> unique_lock(angle_mutex_);
    cv::Vec3f angles = angles_;
    unique_lock.unlock();

    std::cout << "pitch :" << angles[1] << "," << "yaw : " << angles[0] << std::endl;

    if (head_pose == HEAD_FORWARD ::NORMAL) {
        if(angles[1] > 15 or angles[1] < -15 or angles[0] > 15 or angles[0] < -15){
            std::cout << "注册失败,请正视前方重新注册!" << std::endl;
            return false;
        }
    }

    if (head_pose == HEAD_FORWARD ::LEFT) {
        if(angles[1] > -15 /*or angles[0] > 5 or angles[0] < -5*/){
            std::cout << "注册失败,请左转重新注册!" << std::endl;
            return false;
        }
    }

    if (head_pose == HEAD_FORWARD ::RIGHT) {
        if(angles[1] < 15 /*or angles[0] > 5 or angles[0] < -5*/){
            std::cout << "注册失败,请右转重新注册!" << std::endl;
            return false;
        }
    }

    if (head_pose == HEAD_FORWARD ::UP) {
        if(/*angles[1] > 5 or angles[1] < -5 or*/ angles[0] > -5){
            std::cout << "注册失败,请抬头重新注册!" << std::endl;
            return false;
        }
    }

    if (head_pose == HEAD_FORWARD ::DOWN) {
        if(/*angles[1] > 5 or angles[1] < -5 or */angles[0] < 5){
            std::cout << "注册失败,请低头重新注册!" << std::endl;
            return false;
        }
    }

    auto obj_result = object_detect_->Detect(frame);
    if(!obj_result->face()){
        std::cout << "no face detect in currDetectent image!" << std::endl;
        return false;
    }

    face_bbox_ = obj_result->face_bbox();
    cv::Rect bbox = face_bbox_;
    std::vector<cv::Point2f> shape = predictor_->predict(frame, bbox);
    std::vector<cv::Point2f> cv_pts;
    cv::Mat aligned_img;
    cv_pts.emplace_back(shape[44]);
    cv_pts.emplace_back(shape[56]);
    cv_pts.emplace_back(shape[38]);
    cv_pts.emplace_back(shape[60]);
    cv_pts.emplace_back(shape[66]);
    align_method_->set_im(frame);
    align_method_->set_pts(cv_pts);
    aligned_img = face_align_->DoAlign();

    faceid_->GetFaceFeature(aligned_img, feature);

    if(!WriteFeature(feature, feature_path)) {
        std::cout << "[ERROR] : Write feature failed." << std::endl;
        return false;
    }
    std::string image_path = feature_path.substr(0, feature_path.find_last_of(".")) + ".png";
    cv::imwrite(image_path, frame);

    return true;

}

bool TotalFlow::LoadFeature() {
    Features_.clear();
    std::vector<std::string> people_name;
    std::string feature_path = assets_path_ + "/feature/";

    people_name = Listdir(feature_path);

    for(const auto& name:people_name)
    {
        Feature peple_feature;
        peple_feature.name = name;
        name_times_[name] = 0;

        std::vector<std::string> feature_paths = Listfile(feature_path + name);
        for(const auto& fea: feature_paths)
        {
            std::string fea_name = feature_path + name + "/" + fea;
            cv::Mat feature;
            if(!ReadFeature(feature, fea_name)){
                std::cout << "[ERROR]: read feature failed!" << fea_name << std::endl;
                continue;
            }
            peple_feature.features.push_back(feature);
        }
        Features_.push_back(peple_feature);
    }
    name_times_[UnknowFace] = 0;
    return true;
}

std::string TotalFlow::GetName(const cv::Mat &feature) {
    name_times_.clear();
    std::string name = UnknowFace;

    if(feature.empty()) return name;
    for (const auto &Fea: Features_) {
        for (const auto &fea: Fea.features) {
            float score = faceid_->CalcCosScore(fea, feature);
//            std::cout << "score : " << score << std::endl;
            if (score > same_face_thresh) {
                name = Fea.name;
                name_times_[Fea.name]++;
            }
        }
    }

    int max_times = 0;
    for(auto& item: name_times_) {
        if(item.second > max_times){
            name = item.first;
            max_times = item.second;
        }
    }
    return name;
}

std::vector<std::string> TotalFlow::Listdir(const std::string &folder) {
    std::vector<std::string> filenames;
    DIR *dir;
    struct dirent *ptr;
    if((dir = opendir(folder.c_str())) == nullptr){
        std::cout << "Open dir(" << folder << ") error..." << std::endl;
        return filenames;
    }

    while ((ptr = readdir(dir)) != nullptr)
    {
        if((strcmp(ptr->d_name, ".") == 0 ) or
           (strcmp(ptr->d_name, "..") == 0))
            continue;
        if(4 == ptr->d_type) {
            std::string dir_name = ptr->d_name;
            filenames.push_back(ptr->d_name);
        }
    }
    closedir(dir);

    return filenames;
}

std::vector<std::string> TotalFlow::Listfile(const std::string &folder) {
    std::vector<std::string> filenames;
    DIR *dir;
    struct dirent *ptr;
    if((dir = opendir(folder.c_str())) == nullptr){
        std::cout << "Open dir(" << folder << ") error..." << std::endl;
        return filenames;
    }

    while ((ptr = readdir(dir)) != nullptr)
    {
        if((strcmp(ptr->d_name, ".") == 0) or
           (strcmp(ptr->d_name, "..") == 0))
            continue;
        if(8 == ptr->d_type) {
            std::string dir_name = ptr->d_name;
            filenames.push_back(ptr->d_name);
        }
    }
    closedir(dir);

    return filenames;
}

//bool TotalFlow::FaceIDRun(const std::string& registName) {
//    if(feature_name_path_.empty()) {
//        feature_name_path_ = assets_path_ + "/feature/" + registName;
//
//        try {
//            mkdir(feature_name_path_.c_str(), S_IRWXU | S_IRWXO);
//        }
//        catch (...) {
//            std::cout << "[ERROR]: mkdir failed! >>" << feature_name_path_ << std::endl;
//            exit(0);
//        }
//
//    }
//    std::string feature_path = feature_name_path_ + "/" + std::to_string(current_regist_num_) + ".bin";
//    if(!Regist(feature_path)) return false;
//    if(++current_regist_num_>REGISTNUM) regist_over_flag_ = true;
//
//    return true;
//}

bool TotalFlow::FaceIDRun(const std::string& registName, HEAD_FORWARD head_pose) {
    std::string feature_name_path_ = assets_path_ + "/feature/" + registName;
    std::cout << "registName:" << registName << std::endl;
    try {
        if (access(feature_name_path_.c_str(), F_OK) != 0)
        {
            std::cout << "feature_name_path:" << feature_name_path_ << std::endl;
            mkdir(feature_name_path_.c_str(), S_IRWXU | S_IRWXO);
        }
    }
    catch (...) {
        std::cout << "[ERROR]: mkdir failed! >>" << feature_name_path_ << std::endl;
        return false;
    }

    std::cout << "current_regist_num:" << current_regist_num_ << std::endl;
    std::string feature_path = feature_name_path_ + "/" + to_string(current_regist_num_++) + ".bin";

    auto ret = Regist(feature_path, head_pose);

    //将注册的特征加入到缓存的特征向量中。
    if(ret){
        cv::Mat feature;
        //特征缓存中最后一个名字不是当前注册的名字。则创建一个人的特征。
        if(Features_.empty() or Features_[Features_.size()-1].name != registName){
            Feature people_feature;
            people_feature.name = registName;
            Features_.push_back(people_feature);
        }

        //将特征文件加入到注册人的特征向量中。
        if(ReadFeature(feature, feature_path))
            Features_[Features_.size()-1].features.push_back(feature);

        std::lock_guard<std::mutex> name_lock(name_mutex_);
        name_ = registName;
    }
    return ret;
}

bool TotalFlow::WriteFeature(const cv::Mat& feature, const std::string& path) {
    if(feature.empty()) return false;
    fstream fs;
    try {
        fs.open(path, ios::out|ios::binary|ios::trunc);
        fs.write(reinterpret_cast<char*>(feature.data), TotalFlow::FEATURE_LENGTH * sizeof(float));
    }
    catch (...) {
        std::cout << "[ERROR]: Write feature failed! " << path << std::endl;
        fs.close();
        return false;
    }

    fs.close();
    return true;
}

bool TotalFlow::ReadFeature(cv::Mat &feaure, const std::string &path) {
    cv::Mat fea(1,TotalFlow::FEATURE_LENGTH, CV_32FC1);
    fstream fs;
    try {
        fs.open(path, ios::in|ios::binary);
        fs.read(reinterpret_cast<char *>(fea.data), TotalFlow::FEATURE_LENGTH * sizeof(float));
    }
    catch (...) {
        std::cout << "[ERROR]: Read feature failed! " << path << std::endl;
        fs.close();
        return false;
    }

    fs.close();
    feaure = fea.clone();
    return true;
}

TotalFlow::~TotalFlow() {
    keep_running_flag_ = false;
}

bool TotalFlow::Calibration() {
    pitch_base_ = angles_[0];
    yaw_base_ = angles_[1];
    roll_base_ = angles_[2];
    std::cout << "Calibration : " << "yaw_base: " << yaw_base_ <<
              "  pitch_base : " << pitch_base_ << "  roll_base : " << roll_base_ << std::endl;
    return head_pose_detector_.Calibration(yaw_base_, pitch_base_);
}

bool TotalFlow::DistractionSetParam(float time, float rate) {
    std::cout << "Set distraction param. time : " << time << " , rate : " << rate << std::endl;
    auto size = static_cast<size_t>(time * config_.process_fps_);
    auto threshold = static_cast<size_t>(size * rate);
    return distract_judger_.SetParam(size, threshold);
}

bool TotalFlow::DozeSetParam(float time, float rate) {
    std::cout << "Set doze param. time : " << time << " , rate : " << rate << std::endl;
    auto size = static_cast<size_t>(time * config_.process_fps_);
    auto threshold = static_cast<size_t>(size * rate);
    return doze_judger_.SetParam(size, threshold);
}

bool TotalFlow::YawnSetParam(float time, float rate) {
    std::cout << "Set yawn param. time : " << time << " , rate : " << rate << std::endl;
    auto size = static_cast<size_t>(time * config_.process_fps_);
    auto threshold = static_cast<size_t>(size * rate);
    return yawn_judger_.SetParam(size, threshold);
}

bool TotalFlow::SmokeSetParam(float time, float rate) {
    std::cout << "Set smoke param. time : " << time << " , rate : " << rate << std::endl;
    auto size = static_cast<size_t>(time * config_.process_fps_);
    auto threshold = static_cast<size_t>(size * rate);
    return smoke_judger_.SetParam(size, threshold);
}

bool TotalFlow::CallSetParam(float time, float rate) {
    std::cout << "Set call param. time : " << time << " , rate : " << rate << std::endl;
    auto size = static_cast<size_t>(time * config_.process_fps_);
    auto threshold = static_cast<size_t>(size * rate);
    return call_judger_.SetParam(size, threshold);
}

bool TotalFlow::SetOpenMouthThreshold(float threshold) {
    std::cout << "Set open mouth threshold. percent: " << threshold << std::endl;
    threshold = 0.f - log(2/(threshold + 1)-1);
    std::cout << "Set open mouth threshold. threshold : " << threshold << std::endl;
    return eye_mouth_detector_.SetOpenMouthThreshold(threshold);
}

bool TotalFlow::SetLeftEyeThreshold(float threshold) {
    std::cout << "Set left eye threshold. threshold : " << threshold << std::endl;
    return eye_mouth_detector_.SetLeftEyeThreshold(threshold);
}

bool TotalFlow::SetRightEyeThreshold(float threshold) {
    std::cout << "Set right eye threshold. threshold : " << threshold << std::endl;
    return eye_mouth_detector_.SetRightEyeThreshold(threshold);
}

bool TotalFlow::SetDistractionThreshold(float threshold_left, float threshold_right, float threshold_up,
                                        float threshold_down) {
    return head_pose_detector_.SetParam(threshold_left, threshold_right, threshold_up, threshold_down);
}
