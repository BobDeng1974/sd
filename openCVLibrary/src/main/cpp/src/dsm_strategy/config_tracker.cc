#include "config_tracker.h"

bool ConfigTracker::save_image_ = false;

//分神的配置参数
float ConfigTracker::distraction_judge_time_ = 0.f;
float ConfigTracker::distraction_judge_rate_ = 0.f;
int ConfigTracker::distraction_left_angle_ = 0;
int ConfigTracker::distraction_right_angle_ = 0;
int ConfigTracker::distraction_up_angle_ = 0;
int ConfigTracker::distraction_down_angle_ = 0;

//瞌睡检测的配置参数
float ConfigTracker::doze_judge_time_ = 0.f;
float ConfigTracker::doze_judge_rate_ = 0.f;
float ConfigTracker::eye_close_threshold_ = 0.f;

//打哈欠检测的配置参数
float ConfigTracker::yawn_judge_time_ = 0.f;
float ConfigTracker::yawn_judge_rate_ = 0.f;
float ConfigTracker::open_mouth_threshold_ = 0.f;

//抽烟检测的配置参数
float ConfigTracker::smoke_judge_time_ = 0.f;
float ConfigTracker::smoke_judge_rate_ = 0.f;

//打电话的配置参数
float ConfigTracker::call_judge_time_ = 0.f;
float ConfigTracker::call_judge_rate_ = 0.f;


//摄像头是否在线
int ConfigTracker::camera_online_  = 0;

//摄像头ID
int ConfigTracker::camera_index_ = 0;

//摄像头帧率
int ConfigTracker::camera_fps_= 0;

//数据处理帧率
int ConfigTracker::process_fps_= 0;

//路径设置
std::string ConfigTracker::path_load_img_("");
std::string ConfigTracker::send_ip_("127.0.0.1");
std::string ConfigTracker::receive_ip_("127.0.0.1");

bool ConfigTracker::debug_ = false;   //显示landmark和bbox
bool ConfigTracker::use_gaze_ = false;        

ConfigTracker::ConfigTracker(const std::string& tracker_file) : Config(tracker_file) {
    LoadData();
}

void ConfigTracker::LoadData() {

    save_image_ = param<int>("save_image", 0) == 1;

//#分神的配置参数
    distraction_judge_time_ = param<float>("distraction_judge_time", 1.f);
    distraction_judge_rate_ = param<float>("distraction_judge_rate", 100.f);
    distraction_left_angle_ = param<int>("distraction_left_angle", 25);
    distraction_right_angle_ = param<int>("distraction_right_angle", -35);
    distraction_up_angle_ = param<int>("distraction_up_angle", 25);
    distraction_down_angle_ = param<int>("distraction_down_angle", -35);

//#瞌睡检测的配置参数.
    doze_judge_time_ = param<float>("doze_judge_time", 1.f);
    doze_judge_rate_ = param<float>("doze_judge_rate", 100.f);
    eye_close_threshold_ = param<float>("eye_close_threshold", 0.45f);

//#打哈欠检测的配置参数.
    yawn_judge_time_ = param<float>("yawn_judge_time", 1.f);
    yawn_judge_rate_ = param<float>("yawn_judge_rate", 100.f);
    open_mouth_threshold_ = param<float>("open_mouth_threshold", 2.2f);

//#抽烟检测的配置参数.
    smoke_judge_time_ = param<float>("smoke_judge_time", 1.f);
    smoke_judge_rate_ = param<float>("smoke_judge_rate", 100.f);

//#打电话的配置参数.
    call_judge_time_ = param<float>("call_judge_time", 1.f);
    call_judge_rate_ = param<float>("call_judge_rate", 100.f);

//摄像头是否在线
	camera_online_ = param<int>("camera_online",0);

//摄像头ID
    camera_index_ = param<int>("camera_index", 0);

//摄像头帧率
    camera_fps_ = param<int>("camera_fps", 25);

//数据处理帧率
    process_fps_ = param<int>("process_fps", 10);

    use_gaze_ = param<int>("use_gaze", 0) == 1;
//路径设置
//    path_load_img_ = param<std::string>("path_load_img", "");
//    send_ip_ = param<std::string>("send_ip", "127.0.0.1");
//    receive_ip_ = param<std::string>("receive_ip", "127.0.0.1");

    debug_ = param<int>("debug", 0) == 1;
}
