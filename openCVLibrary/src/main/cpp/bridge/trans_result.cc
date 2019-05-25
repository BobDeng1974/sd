#include "trans_result.h"

//#include "zmq.h"
#include <assert.h>
#include <stdlib.h>

const int SENDING_RECEIVING_PORT = 6001;

Sending::Sending(std::string addr){
//    context_ = zmq_ctx_new();
//    push_ = zmq_socket(context_,ZMQ_PUSH);
//    int ret = -1;
//
//    int send_hwm = 1;
//    size_t option_lenght = sizeof(send_hwm);
//    zmq_setsockopt(push_, ZMQ_SNDHWM, &send_hwm, option_lenght);
//    zmq_getsockopt(push_, ZMQ_SNDHWM, &send_hwm, &option_lenght);
//    assert(1 == send_hwm && "set hwm failed.");
//
//    std::string address = "tcp://"+addr+":"+std::to_string(SENDING_RECEIVING_PORT);
//    ret = zmq_connect(push_,address.c_str());
//    assert(ret == 0 && "bind port had been taken.");
}

Sending::~Sending(){
//    if(push_ != nullptr){
//        zmq_close(push_);
//        zmq_ctx_destroy(context_);
//        push_ = nullptr;
//    }
}

void Sending::Send(Result &result) {
//    zmq_msg_t msg;
//    std::string sending_data = result.SerializeAsString();
//    zmq_msg_init_size(&msg,sending_data.size());
//    void* msg_data_addr = zmq_msg_data(&msg);
//    memcpy(msg_data_addr,sending_data.c_str(),result.ByteSize());
//    int ret = zmq_msg_send(&msg,push_,0);
//    assert(ret != -1 && "sending failed");
//    zmq_msg_close(&msg);
}

Receiving::Receiving() {
//    context_ = zmq_ctx_new();
//    pull_ = zmq_socket(context_,ZMQ_PULL);
//    int ret = -1;
//
//    std::string address = "tcp://*:"+std::to_string(SENDING_RECEIVING_PORT);
//    ret = zmq_bind(pull_,address.c_str());
//    assert(ret == 0 && "bind port had been taken.");
}

Receiving::~Receiving() {
//    if(pull_ != nullptr){
//        zmq_close(pull_);
//        zmq_ctx_destroy(context_);
//        pull_ = nullptr;
//    }
}

void Receiving::Recv(Result &recv) {
//    zmq_msg_t msg;
//    zmq_msg_init(&msg);
//    int size = zmq_msg_recv(&msg,pull_,0);
//    if(size != -1) {
//        void* data = zmq_msg_data(&msg);
//        size_t size_of_data = zmq_msg_size(&msg);
//        std::string received_data;
//        received_data.resize(size_of_data);
//        memcpy(const_cast<char*>(received_data.c_str()),data,size_of_data*sizeof(char));
//        bool succ = recv.ParseFromString(received_data);
//        if(succ) {
//            std::cout << "parse success" << std::endl;
//        }
//        else {
//            std::cout << "parse failed" << std::endl;
//        }
//    }
//    zmq_msg_close(&msg);
}


void PackageHelper::SetMat(Result& result,cv::Mat& mat){
    std::string* image = result.mutable_image();
    image->resize(mat.total()*mat.elemSize());
    memcpy(const_cast<char*>(image->c_str()),mat.data,sizeof(char)*image->length());
}

void PackageHelper::GetMat(Result& result,cv::Mat& out){
    std::string image_data = result.image();
    memcpy(out.data,image_data.c_str(),sizeof(char)*image_data.size());
}


void PackageHelper::SetLandmarks(Result &result, std::vector<cv::Point2f> &points) {
    result.clear_landmarks();
    for(size_t i=0;i<points.size();i++) {
        Point2f* p = result.add_landmarks();
        p->set_x(points.at(i).x);
        p->set_y(points.at(i).y);
    }
}
void PackageHelper::SetRect(Rect* result,cv::Rect& bb) {
    result->set_x(bb.x);
    result->set_y(bb.y);
    result->set_width(bb.width);
    result->set_height(bb.height);
}

void PackageHelper::SetPoint3f(Point3f* point3f, const cv::Point3f& point) {
    point3f->set_x(point.x);
    point3f->set_y(point.y);
    point3f->set_z(point.z);
}
void PackageHelper::SetPoint2f(Point2f* point2f, const cv::Point2f& point) {
    point2f->set_x(point.x);
    point2f->set_y(point.y);
}

void PackageHelper::GetLandmarks(Result& result,std::vector<cv::Point2f>& points) {
    std::vector<cv::Point2f>(result.landmarks_size()).swap(points);
    for(int i=0;i<result.landmarks_size();i++) {
        Point2f point = result.landmarks(i);
        points.at(i) = cv::Point2f(point.x(),point.y());
    }
}

void PackageHelper::GetRect(const Rect& result,cv::Rect& out) {
    out.x = result.x();
    out.y = result.y();
    out.width = result.width();
    out.height =  result.height();
}
void PackageHelper::GetPoint3f(const Point3f& point3f,cv::Point3f& out) {
    out.x = point3f.x();
    out.y = point3f.y();
    out.z = point3f.z();
}
void PackageHelper::GetPoint2f(const Point2f& point2f,cv::Point2f& out) {
    out.x = point2f.x();
    out.y = point2f.y();
}

void PackageHelper::SetQuaternion(Quaternion *quaternion, cv::Vec4d &vec4d) {
    quaternion->set_w(static_cast<float>(vec4d[0]));
    quaternion->set_x(static_cast<float>(-vec4d[1]));
    quaternion->set_y(static_cast<float>(vec4d[3]));
    quaternion->set_z(static_cast<float>(vec4d[2]));
}

void PackageHelper::SetDistraction(Result &result, int type) {
    switch (type){
        case 0:
            result.set_distraction(DISTRACTION_TYPE::NORMAL);
            break;
        case 1:
            result.set_distraction(DISTRACTION_TYPE::LEFT);
            break;
        case 2:
            result.set_distraction(DISTRACTION_TYPE::RIGHT);
            break;
        case 3:
            result.set_distraction(DISTRACTION_TYPE::UP);
            break;
        case 4:
            result.set_distraction(DISTRACTION_TYPE::DOWN);
            break;
        default:
            std::cerr << "type : " << type << std::endl;
            assert(false and "distraction wrong type");
            break;
    }

}


