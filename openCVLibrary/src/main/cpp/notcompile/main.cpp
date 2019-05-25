#include <iostream>
#include "src/camera/cameras.h"
#include <memory>
#include <unistd.h>
#include <chrono>
#include <trans_settings.h>
#include "../total_flow.h"
#include "config_tracker.h"

int main(int argc, char **argv) {

    ConfigTracker config_("../assets/configure.yaml");
    cv::Mat frame;
    Result result;

    Sending sending(config_.send_ip_);
    ReceivingSettings receiving(config_.receive_ip_);

    std::shared_ptr <Camera> cam_;

    if (config_.camera_online_ == 0)
        cam_ = std::make_shared<CameraOffLine>(config_.path_load_img_);
    else
        cam_ = std::make_shared<CameraOnLine>(config_.camera_index_);

    TotalFlow total_flow("../assets");

    receiving.RegistFaceID(std::bind(&TotalFlow::FaceIDRun, &total_flow,
                                     std::placeholders::_1, std::placeholders::_2));

    receiving.RegistCalib(std::bind(&TotalFlow::Calibration, &total_flow));

    receiving.RegistDoze(std::bind(&TotalFlow::DozeSetParam, &total_flow,
                                   std::placeholders::_1, std::placeholders::_2));

    receiving.RegistYawn(std::bind(&TotalFlow::YawnSetParam, &total_flow,
                                   std::placeholders::_1, std::placeholders::_2));

    receiving.RegistSmoke(std::bind(&TotalFlow::SmokeSetParam, &total_flow,
                                   std::placeholders::_1, std::placeholders::_2));

    receiving.RegistCall(std::bind(&TotalFlow::CallSetParam, &total_flow,
                                    std::placeholders::_1, std::placeholders::_2));

    receiving.RegistLeftEye(std::bind(&TotalFlow::SetLeftEyeThreshold, &total_flow,
                                      std::placeholders::_1));

    receiving.RegistRightEye(std::bind(&TotalFlow::SetRightEyeThreshold, &total_flow,
                                      std::placeholders::_1));

    receiving.RegistMouth(std::bind(&TotalFlow::SetOpenMouthThreshold, &total_flow,
                                      std::placeholders::_1));
    while (true) {
        cam_->Read(frame);
        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

//        cv::imshow("frame", frame);
//        cv::waitKey(1);

        total_flow.Run(frame, result);
        cv::cvtColor(frame, frame, CV_BGR2GRAY);
        PackageHelper::SetMat(result, frame);
        sending.Send(result);
        result.set_fps(config_.camera_fps_);
        result.set_age(666);
        result.set_cpu_utilization(0.55);
        result.set_mem_occupation(50);
        result.set_gender(GENDER::MALE);

        if(config_.debug_) {
            if (result.have_face()) {
                std::vector<cv::Point2f> landmarks;
                PackageHelper::GetLandmarks(result, landmarks);

                cv::Rect face_bbox;
                PackageHelper::GetRect(result.face_bbox(), face_bbox);

                std::cout << "name              :" << result.name() << std::endl;
                std::cout << "have_face         :" << result.have_face() << std::endl;
                std::cout << "face_bbox         :" << result.face_bbox().x() << " "
                          << result.face_bbox().y() << " "
                          << result.face_bbox().width() << " "
                          << result.face_bbox().height() << std::endl;
                std::cout << "landmarks         :" << result.landmarks(0).x() << " " << result.landmarks(0).y()
                          << std::endl;
                std::cout << "head_pose         :" << result.head_pose().x() << " "
                          << result.head_pose().y() << " "
                          << result.head_pose().z() << std::endl;
                std::cout << "quaternion        :" << result.head_quaternion().w() << " "
                          << result.head_quaternion().x() << " "
                          << result.head_quaternion().y() << " "
                          << result.head_quaternion().z() << std::endl;
                std::cout << "doze              :" << result.doze() << std::endl;
                std::cout << "yawn              :" << result.yawn() << std::endl;
                std::cout << "phone             :" << result.phone() << std::endl;
                std::cout << "smoking           :" << result.smoking() << std::endl;

                std::cout << "phone_bbox        :" << result.phone_bbox().x() << " "
                          << result.phone_bbox().y() << "  "
                          << result.phone_bbox().width() << " "
                          << result.phone_bbox().height() << std::endl;

                std::cout << "cigarette_bbox    :" << result.cigarette_bbox().x() << " "
                          << result.cigarette_bbox().y() << "  "
                          << result.cigarette_bbox().width() << " "
                          << result.cigarette_bbox().height() << std::endl;

                std::cout << "have_mouth        :" << result.have_mouth() << std::endl;
                std::cout << "mouth_open        :" << result.mouth_open() << std::endl;
                std::cout << "mouth_open_rate   :" << result.mouth_open_rate() << std::endl;

                std::cout << "left_eye_open     :" << result.left_eye_open() << std::endl;
                std::cout << "right_eye_open    :" << result.right_eye_open() << std::endl;
                std::cout << "have_left_eye     :" << result.have_left_eye() << std::endl;
                std::cout << "have_right_eye    :" << result.have_right_eye() << std::endl;
                std::cout << "left_open_rate    :" << result.left_eye_open_rate() << std::endl;
                std::cout << "right_open_rate   :" << result.right_eye_open_rate() << std::endl;
                std::cout << "left_direction    :" << result.left_eye_direction().x() << " "
                          << result.left_eye_direction().y() << " "
                          << result.left_eye_direction().z() << std::endl;

                std::cout << "right_direction   :" << result.right_eye_direction().x() << " "
                          << result.right_eye_direction().y() << " "
                          << result.right_eye_direction().z() << std::endl;

                std::cout << "left_eye_center   :" << result.left_eye_center().x() << " "
                          << result.left_eye_center().y() << " "
                          << result.left_eye_center().z() << std::endl;

                std::cout << "right_eye_center  :" << result.right_eye_center().x() << " "
                          << result.right_eye_center().y() << " "
                          << result.right_eye_center().z() << std::endl;

                std::cout << "camera_avaliable  :" << result.camera_avaliable() << std::endl;
                std::cout << "age               :" << result.age() << std::endl;
                std::cout << "fps               :" << result.fps() << std::endl;
                std::cout << "cpu_utilization   :" << result.cpu_utilization() << std::endl;
                std::cout << "mem_occupation    :" << result.mem_occupation() << std::endl;
                std::cout << "gender            :" << result.gender() << std::endl;
                std::cout << "\n\n" << std::endl;
            }
        }
    }

    return 0;
}

