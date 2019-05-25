#ifndef DSM_STRATEGY_CONFIG_TRACKER_H
#define DSM_STRATEGY_CONFIG_TRACKER_H

#include "utils/file/config.h"

using namespace untouch_utils;


/*! \brief 定义访问配置文件的接口
 *
 * 用于访问配置文件中的各种数据，需要在该类内定义每种配置参数的类型为static,
 * 然后再调用LoadData()从配置文件中读取各个配置参数的值
 */
class ConfigTracker : public Config {
public:
    /// \brief 构造函数
    /// \param [in] tracker_file 配置文件的路径
    ConfigTracker(const std::string &tracker_file);

    /// \brief 加载配置文件内容
    void LoadData() ;

public:
//For LiaoYZ
    static bool save_image_;

    //分神的配置参数
    static float distraction_judge_time_;
    static float distraction_judge_rate_;
    static int distraction_left_angle_;
    static int distraction_right_angle_;
    static int distraction_up_angle_;
    static int distraction_down_angle_;

    //瞌睡检测的配置参数
    static float doze_judge_time_;
    static float doze_judge_rate_;
    static float eye_close_threshold_;

    //打哈欠检测的配置参数
    static float yawn_judge_time_;
    static float yawn_judge_rate_;
    static float open_mouth_threshold_;


    //抽烟检测的配置参数
    static float smoke_judge_time_;
    static float smoke_judge_rate_;

    //打电话的配置参数
    static float call_judge_time_;
    static float call_judge_rate_;

    static int camera_online_;  // 摄像头是否在线
    static int camera_index_;   // 摄像头ID
    static int camera_fps_;            // 摄像头帧率
    static int process_fps_;           // 数据处理帧率

	//路径配置
	static std::string path_load_img_; // 加载视频图片的路径
    static std::string send_ip_;
    static std::string receive_ip_;

    static bool debug_;    //显示landmark和bbox
    static bool use_gaze_;
};


#endif //DSM_STRATEGY_CONFIG_TRACKER_H
