//
// Created by untouch.
//
/*! \file TotalFlow.h
 *  \brief 定义程序的主流程
 *
 *  该文件为程序的入口,并且定义了程序的整体流程
 */

#ifndef STRATEGY_TOTALFLOW_H
#define STRATEGY_TOTALFLOW_H

#include <iostream>
#include <chrono>
#include <algorithm>
#include <thread>
#include <opencv2/opencv.hpp>
#include <mutex>

#include "shape_predict.h"
#include "align_method.h"
#include "faceAlign.h"
//#include "gaze_predict.h"
#include "faceid_ncnn.h"
#include "faceAttribute.h"
#include "align_method.h"
#include "faceAlign.h"
#include "head_pose_estimator.h"
#include "object_detection.h"

#include "src/dsm_strategy/config_tracker.h"
#include "src/dsm_strategy/judger.h"

#include "src/dsm_strategy/head_pose_states.h"
#include "src/dsm_strategy/eye_mouth_plus.h"
#include "src/dsm_strategy/utils/time/time_stamp.h"

#include "trans_result.h"

using namespace std;
using namespace cv;

enum class HEAD_FORWARD{
    NORMAL= 0,
    LEFT,
    RIGHT,
    UP,
    DOWN
};


class TotalFlow {
public:
    /// \brief 构造函数
    explicit TotalFlow(const std::string& path);
    ~TotalFlow();

    /// \brief 程序运行入口
    void Run(cv::Mat& frame, Result& result, bool regist = false, const std::string& name = "");

    bool FaceIDRun(const std::string& name, HEAD_FORWARD head_pose);

    bool Calibration();
    bool DistractionSetParam(float time, float rate);
    bool DozeSetParam(float time, float rate);
    bool YawnSetParam(float time, float rate);
    bool SmokeSetParam(float time, float rate);
    bool CallSetParam(float time, float rate);
    bool SetOpenMouthThreshold(float threshold);
    bool SetLeftEyeThreshold(float threshold);
    bool SetRightEyeThreshold(float threshold);
    bool SetDistractionThreshold(float threshold_left, float threshold_right,
                                 float threshold_up, float threshold_down);

private:
    /// \brief 用于处理图像的线程
    void ProcessImageThread();

    /// \brief 处理程序的函数
    void RunProcess();

    /// \brief 运行驾驶员检测策略的主函数
    /// \details 不同的检测策略之间的优先级从高到低依次为：分神检测，疲劳检测，吸烟检测，打电话检测，可疑行为检测，
    /// 聊天检测。检测策略按照优先级依次执行，只有当某个策略为正常状态时，才继续执行下一个检测策略。
    void RunMainStep();


    /// \brief 检测图像中的landmark,香烟,手掌,打电话,头的旋转角度，获取的数据用于检测驾驶员状态
    /// \param [in] frame 要检测的图像
    /// \retval 检测是否成功
    bool DetectFrame(const Mat &frame);

private:
    std::string assets_path_;
    std::string config_path_;
    std::string sp_model_path_;
    std::string head_pose_path_;
    std::string faceid_path_;
    std::string gaze_model_path_;
    std::string object_model_path_;

    std::shared_ptr<AlignMethod> align_method_;
    std::shared_ptr<FaceAlign> face_align_;
//    std::shared_ptr<GazePredict> gaze_predict_;
    std::shared_ptr<ShapePredictor> predictor_;
    std::shared_ptr<HeadPoseEstimator> head_pose_estimator_;
    std::shared_ptr<FaceIDNCNN> faceid_;
    std::shared_ptr<ObjectDetect> object_detect_;

    ConfigTracker config_;
    HeadPoseStatus head_pose_detector_;
    EyeMouthStatusPlus eye_mouth_detector_;

    bool first_time_flage_;
    bool keep_running_flag_;
    bool smoke_state_;
    bool call_state_;

    float yaw_base_;
    float pitch_base_;
    float roll_base_;

    std::mutex frame_mutex_;
    std::mutex bbox_mutex_;
    std::mutex landmark_mutex_;
    std::mutex angle_mutex_;
    std::mutex result_mutex_;

    Judger smoke_judger_;
    Judger call_judger_;
    Judger doze_judger_;
    Judger yawn_judger_;
    Judger distract_judger_;

    /// 以下为在图像中检测到的有效信息,用于各种策略判断
    cv::Mat frame_;     ///< 摄像头读取的帧
    cv::Rect face_bbox_;    ///<人脸bbox
    cv::Vec3f angles_;    ///<检测到的人脸角度
    cv::Vec3d gaze_direction_;
    cv::Vec3d HCSOrient_;
    cv::Point3d left_eye_center_;
    cv::Point3d right_eye_center_;
    cv::Vec4d quaternion_;     ///< 头部姿态四元数。
    cv::Rect smoke_bbox_;
    cv::Rect call_bbox_;
    vector<cv::Point2f> landmarks_; ///< 检测到的人脸landmarks
    Result result_;

    std::thread process_image_thread_;

private:
    int REGISTNUM = 10;
    int current_regist_num_;
    bool regist_over_flag_;
    bool mark_no_face_;
    bool face_match_flag_;
    size_t face_match_count_;
//    std::string feature_name_path_;
    std::string name_;
    std::mutex name_mutex_;
    struct Feature{
        std::string name;
        std::vector<cv::Mat> features;
    };
    bool Regist(const std::string& name, HEAD_FORWARD head_pose);
    bool LoadFeature();
    bool WriteFeature(const cv::Mat& feature, const std::string& path);
    bool ReadFeature(cv::Mat &feaure, const std::string& path);

    std::string GetName(const cv::Mat& feature);
    std::vector<std::string>Listdir(const std::string& folder);
    std::vector<std::string>Listfile(const std::string& folder);

    static const int FEATURE_LENGTH;
    static const float same_face_thresh;
    static const std::string UnknowFace;
    static const std::string NoFace;

    std::vector<Feature> Features_;
    std::unordered_map<std::string, int>name_times_;
};


#endif //STRATEGY_TOTALFLOW_H
