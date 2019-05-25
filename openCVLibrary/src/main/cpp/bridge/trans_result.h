#pragma once

#include <string>
#include "result2u3d.pb.h"
#include <opencv2/opencv.hpp>

class Sending {
public:
    explicit Sending(std::string addr);
    ~Sending();
    void Send(Result& result);
private:
    void *context_,*push_;
};


class Receiving {
public:
    Receiving();
    ~Receiving();
    void Recv(Result& recv);
private:
    void *context_,*pull_;
};


class PackageHelper{
public:
    static void SetMat(Result& result,cv::Mat& mat);
    static void SetLandmarks(Result &result, std::vector<cv::Point2f> &points);
    static void SetRect(Rect* result,cv::Rect& bb);
    static void SetPoint3f(Point3f* point3f, const cv::Point3f& point);
    static void SetPoint2f(Point2f* point2f, const cv::Point2f& point);
    static void SetQuaternion(Quaternion* quaternion, cv::Vec4d& vec4d);
    static void SetDistraction(Result& result, int type);

    /**
     * Get Mat from proto message Result. **NOTE** this cv::Mat must be allocated before.
     * @param result
     * @param out
     */
    static void GetMat(Result& result,cv::Mat& out);
    static void GetLandmarks(Result& result,std::vector<cv::Point2f>& points);
    static void GetRect(const Rect& result,cv::Rect& out);
    static void GetPoint3f(const Point3f& point3f,cv::Point3f& out);
    static void GetPoint2f(const Point2f& point2f,cv::Point2f& out);
};