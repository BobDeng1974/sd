//
// Created by zhaoyg on 19-4-10.
//

#ifndef NRMALIZATION_NRMAL_H
#define NRMALIZATION_NRMAL_H

#include<math.h>
#include<vector>
#include<utility>
#include <iostream>
#include<opencv2/opencv.hpp>

typedef struct NrmParams
{
    cv::Mat facemodelLM;
    cv::Mat facemodelEPnP;
    cv::Mat ProjMatrix;
    cv::Matx33d ProjMatrix1;
    cv::Mat invProjMatrix;
    cv::Matx33d invProjMatrix1;
    cv::Mat distCoef1;
    cv::Mat VirIntrinsic;
    cv::Matx33d VirIntrinsic1;
    float vrDistance=-1.0;
    float drawLength=-1.0;
}NrmParams;

void mkParameters(std::string xmlpath,NrmParams&params);
void readlandmarks(std::string& filename,std::vector<cv::Point2f>&landmarks);
void mkColorList(std::vector<cv::Scalar>&colList);

static void pltDirection(cv::Mat& CCSIm,const cv::Point3d&Ori,
                         cv::Vec3d& xDirection,const NrmParams&params)
{
    using namespace cv;
    Vec3d tempOri(Ori);
    Vec3d start3D=tempOri/tempOri[2];
    Vec3d endPt3D=tempOri+xDirection*params.drawLength;
    endPt3D=endPt3D/endPt3D[2];
    Vec3d Start2D=params.ProjMatrix1*start3D;
    Vec3d End2D=params.ProjMatrix1*endPt3D;
    Point startPt=Point(int(round(Start2D[0])),int(round(Start2D[1])));
    Point endPt=Point(int(round(End2D[0])),int(round(End2D[1])));
    cv::line(CCSIm,startPt,endPt,Scalar(255,0,255),2);
}



class DataNrmalization
{
public:
    DataNrmalization(std::string cfgpath)
    { mkParameters(cfgpath,params); };

//    inline Matx31f unNrmalOrient(Vec2d& angles,Matx33d&MxR)
//    { Matx31f xDirection=RxSR2Orient(angles,MxR);
//        return xDirection;};

    /**
     * This method will output left-hand coordinate vec3d
     * @param anges [in]
     * @param HCSOrient [out] head coordinate used by Unity3d.
     * @return camera coordinate vect3d of eye gaze.
     */
    cv::Vec3d unNrmalOrient(cv::Vec2d& angles,cv::Vec3d& HCSOrient) const;


    void mkNormalImGz(const cv::Mat&CCSIm,const std::vector<cv::Point2f>&landmark);
    void mkTransformMat2(cv::Matx33d&rotate, cv::Vec3d&translatation);
    void QueryNrmImage(cv::Mat&NrmMat)const
    {
        ////cv::imshow("normed",_tempNrmImage_);
        ////cv::waitKey(10);
        _tempNrmImage_.copyTo(NrmMat);
    };
    void wrtIntermediate(const std::string& rtpath,int frmidx)const;
    void pltVisualDirect(cv::Mat& CCSIm,cv::Vec3d& xDirection)const
    {
        pltDirection(CCSIm,_leftEyeCt_, xDirection,params);
        pltDirection(CCSIm,_rightEyeCt_, xDirection,params);
    }

    cv::Point3d GetLeftEyeCt(){
        return _leftEyeCt_;
    }

    cv::Point3d GetRightEyeCt(){
        return _rightEyeCt_;
    }




    ~DataNrmalization(){};
    void wrtAuxInfo(std::string &pathFirst,std::vector<cv::Point2f>&shape)
    {
        using namespace std;
        {
            ofstream fObj(pathFirst+"_faceCt.txt",std::ios::out);
            fObj<<_faceCt_.x<<","<<_faceCt_.y<<","<<_faceCt_.z<<endl;
            fObj.close();
        }

        {
            ofstream fObj(pathFirst+"_MR.txt",std::ios::out);
            fObj<<_MR_(0,0)<<","<<_MR_(0,1)<<","<<_MR_(0,2)<<endl;
            fObj<<_MR_(1,0)<<","<<_MR_(1,1)<<","<<_MR_(1,2)<<endl;
            fObj<<_MR_(2,0)<<","<<_MR_(2,1)<<","<<_MR_(2,2)<<endl;
            fObj.close();
        }

        {
            ofstream fObj(pathFirst+"_MSR.txt",std::ios::out);
            fObj<<_MSR_(0,0)<<","<<_MSR_(0,1)<<","<<_MSR_(0,2)<<endl;
            fObj<<_MSR_(1,0)<<","<<_MSR_(1,1)<<","<<_MSR_(1,2)<<endl;
            fObj<<_MSR_(2,0)<<","<<_MSR_(2,1)<<","<<_MSR_(2,2)<<endl;
            fObj.close();
        }

        {
            ofstream fObj(pathFirst+"_S2DIMxRot.txt",std::ios::out);
            fObj<<_S2DIMxRot_(0,0)<<","<<_S2DIMxRot_(0,1)<<","<<_S2DIMxRot_(0,2)<<endl;
            fObj<<_S2DIMxRot_(1,0)<<","<<_S2DIMxRot_(1,1)<<","<<_S2DIMxRot_(1,2)<<endl;
            fObj<<_S2DIMxRot_(2,0)<<","<<_S2DIMxRot_(2,1)<<","<<_S2DIMxRot_(2,2)<<endl;
            fObj.close();
        }


        {
            ofstream fObj(pathFirst+"_HpRotation.txt",std::ios::out);
            fObj<<_HpRotation_(0,0)<<","<<_HpRotation_(0,1)<<","<<_HpRotation_(0,2)<<endl;
            fObj<<_HpRotation_(1,0)<<","<<_HpRotation_(1,1)<<","<<_HpRotation_(1,2)<<endl;
            fObj<<_HpRotation_(2,0)<<","<<_HpRotation_(2,1)<<","<<_HpRotation_(2,2)<<endl;
            fObj.close();
        }


        {
            ofstream fObj(pathFirst+"_leftEyeCt.txt",std::ios::out);
            fObj<<_leftEyeCt_.x<<","<<_leftEyeCt_.y<<","<<_leftEyeCt_.z<<endl;
            fObj.close();
        }
        {
            ofstream fObj(pathFirst+"_rightEyeCt.txt",std::ios::out);
            fObj<<_rightEyeCt_.x<<","<<_rightEyeCt_.y<<","<<_rightEyeCt_.z<<endl;
            fObj.close();
        }

        {
            ofstream fObj(pathFirst+"_shape.txt",std::ios::out);
            for(size_t ptidx=0;ptidx<shape.size();ptidx++)
            fObj<<shape[ptidx].x<<","<<shape[ptidx].y<<endl;
            fObj.close();
        }


        cv::imwrite(pathFirst+".bmp",_tempNrmImage_);
    }


private:
    NrmParams params;
private:
    cv::Point3d _faceCt_;
    cv::Point3d _leftEyeCt_,_rightEyeCt_;
    cv::Mat _tempNrmImage_;
    cv::Matx33d _MR_,_MSR_,_S2DIMxRot_;
    cv::Matx33d _HpRotation_,_TransHpRotation_;
    std::vector<cv::Scalar> colList;
};

#endif //NRMALIZATION_NRMAL_H