//
// Created by zhaoyg on 19-4-10.
//
#include<fstream>
#include "Nrmal.h"

using namespace cv;
using namespace std;

#define __WillTest_
#undef __WillTest_
void mkParameters(string xmlpath,NrmParams&params)
{
    FileStorage fs(xmlpath,cv::FileStorage::READ);
    cout<<"isOpend:"<<fs.isOpened()<<endl;
    if(!fs.isOpened())
        return;

    fs["vrCam"]>>params.VirIntrinsic;
    fs["facemodel"]>>params.facemodelLM;
    fs["projmatrix"]>>params.ProjMatrix;
    params.vrDistance=fs["vrDistance"];
    params.drawLength=float(fs["drawLength"]);


    params.facemodelEPnP=params.facemodelLM.clone();
    params.facemodelEPnP=params.facemodelEPnP.reshape(3,params.facemodelLM.rows);

    //bool isEqual1=(params.ProjMatrix.type()==CV_32F);
    //bool isEqual2=(params.ProjMatrix.type()==CV_64F);

    params.invProjMatrix=params.ProjMatrix.inv();
    params.distCoef1.create(1,4,CV_64F);
    params.distCoef1.setTo(Scalar(0,0,0));

    for(int row=0;row<3;row++)
        for(int col=0;col<3;col++)
        {
            params.ProjMatrix1(row,col)=params.ProjMatrix.at<double>(row,col);
            params.invProjMatrix1(row,col)=params.invProjMatrix.at<double>(row,col);
            params.VirIntrinsic1(row,col)=params.VirIntrinsic.at<double>(row,col);
        }
    fs.release();
}


static void vcNormalize(Vec3d& indata)
{
    double xnorm=0.0;
    for(size_t idx=0;idx<3;idx++)
        xnorm+=(indata(idx)*indata(idx));
    xnorm=sqrt(double(xnorm));
    for(size_t idx=0;idx<3;idx++)
        indata(idx)=indata(idx)/xnorm;
}


static Vec3d mkOutProduct(Vec3d&v1,Vec3d&v2)
{
    Vec3d out(0.0,0.0,0.0);
    out(0)=v1[1]*v2[2]-v1[2]*v2[1];
    out(1)=v1[2]*v2[0]-v1[0]*v2[2];
    out(2)=v1[0]*v2[1]-v1[1]*v2[0];
    return out;
}

static Vec2d HPose3DTo2D(Vec3d&rvecs)
{   Matx33d rmat;
    Rodrigues(rmat,rvecs);
    double theta =asin(rmat(1,2));
    double phi=atan2(rmat(0,2),rmat(2,2));
    return Vec2d(theta,phi);
}

static Vec2d CvtOrient2Agl(Vec3d &Orientation)
{
    double theta =asin(-1.0*Orientation(1));
    double phi = atan2(-1.0*Orientation(0), -1.0*Orientation(2));
    return Vec2d(theta,phi);
}

static inline Vec3d Angle2Orient(Vec2d &angles)
{
    return Vec3d(-1.0*cos(angles[0])*sin(angles[1]),
                 -1.0*sin(angles[0]),
                 -1.0*cos(angles[0])*cos(angles[1]));
}

static Matx31d RxSR2Orient(Vec2d& angles,const Matx33d&MxR)
{
    Matx31d SrcDirection;
    Vec3d Orient=Angle2Orient(angles);
    Matx31d NrmalOrient(Orient[0],Orient[1],Orient[2]);
    solve(MxR, NrmalOrient,SrcDirection);
    return SrcDirection;
}


static Matx33d mkVrRotate(Matx33d&rotate,Vec3d&translatation)
{
    Vec3d virZCCS(translatation);
    vcNormalize(virZCCS);

    Vec3d HeadXCCS({rotate(0,0),rotate(1,0),rotate(2,0)});
    Vec3d virYCCS=mkOutProduct(virZCCS, HeadXCCS);
    vcNormalize(virYCCS);
    Vec3d virXCCS=mkOutProduct(virYCCS, virZCCS);
    vcNormalize(virXCCS);

    Matx33d vrRotate(virXCCS[0],virYCCS[0],virZCCS[0],
                     virXCCS[1],virYCCS[1],virZCCS[1],
                     virXCCS[2],virYCCS[2],virZCCS[2]);
    return vrRotate;
}


static void mkLabelRxSR(Vec3d& SrcDirection,Matx33f&MxSR,Matx33f&MxR,
                        Vec2d& SrcGzAgl, Vec2d& MxSRGzAgl,Vec2d&MxRGzAgl)
{
    Vec3d SrcTemp(SrcDirection);
    vcNormalize(SrcTemp);
    SrcGzAgl=CvtOrient2Agl(SrcTemp);

    Vec3d MxSRGzDirection=MxSR*SrcDirection;
    vcNormalize(MxSRGzDirection);
    MxSRGzAgl = CvtOrient2Agl(MxSRGzDirection);


    Vec3d MxRGzDirection = MxR*SrcDirection;
    vcNormalize(MxRGzDirection);
    MxRGzAgl = CvtOrient2Agl(MxRGzDirection);
}




static void mkHPoseRxSR(Matx33d&SrcRotation,
                        Matx33d&MxSR,Matx33d&MxR,
                        Vec2d&MxSRxHAgl2D,Vec2d&MxRxHAgl2D)
{
    Vec3d MxSRxHAgl,MxRxHAgl;
    Matx33d MxSRxHRotation=MxSR*SrcRotation;
    Matx33d MxRxHRotation = MxR*SrcRotation;
    Rodrigues(MxSRxHRotation,MxSRxHAgl);
    Rodrigues(MxRxHRotation,MxRxHAgl);
    MxSRxHAgl2D=HPose3DTo2D(MxSRxHAgl);
    MxRxHAgl2D=HPose3DTo2D(MxRxHAgl);
}


static void mkCoordRange2(Size SrcSz,Size DstSz, Matx33d&TransSxy2Dxy,
                          vector<Point>&srcmesh, vector<Point>&desmesh)
{
    Mat srcmesh0(3,DstSz.width*DstSz.height,CV_64F);
    Mat desmesh0(3,DstSz.width*DstSz.height,CV_64F);

    for(int row=0,pt=0;row<DstSz.height;row++)
        for(int col=0;col<DstSz.width;col++,pt++)
        {
            desmesh0.at<double>(0,pt)=col;
            desmesh0.at<double>(1,pt)=row;
            desmesh0.at<double>(2,pt)=1;
        }

    solve(TransSxy2Dxy,desmesh0,srcmesh0);
    assert(srcmesh0.type()==CV_64F);

    srcmesh.clear();
    desmesh.clear();
    int validcnt=0,unvalid=0;
    int srcwidth=SrcSz.width-1,srcheight=SrcSz.height-1;
    for(int row=0,pt=0;row<DstSz.height;row++)
        for(int col=0;col<DstSz.width;col++,pt++)
        {
            float x=srcmesh0.at<double>(0,pt)/srcmesh0.at<double>(2,pt);
            float y=srcmesh0.at<double>(1,pt)/srcmesh0.at<double>(2,pt);

            if(x>=0 && x<=srcwidth && y>=0 && y<=srcheight)
            {
                srcmesh.push_back(Point(int(round(x)),int(round(y))));
                desmesh.push_back(Point(col,row));
            }
        }
}

static void TransformImage2(Mat&CCSIm,Matx33d&Transform,Matx33d&vrCam,Mat& DstIm)
{
    Size SrcSz(CCSIm.cols,CCSIm.rows);
    Size DstSz(int(round(vrCam(0,2)*2+1)),int(round(vrCam(1,2)*2+1)));
#ifdef __WillTest_
    cout<<"Transform-----"<<endl;
    for(int row=0;row<3;row++)
            cout<<Transform(row,0)<<","<<Transform(row,1)<<","<<Transform(row,2)<<endl;
#endif
    vector<Point>srcmesh,desmesh;
    mkCoordRange2(SrcSz,DstSz, Transform,srcmesh, desmesh);

    DstIm.create(DstSz.height,DstSz.width,CV_8UC3);
    DstIm.setTo(Scalar(0,0,0));
    for(size_t pt=0;pt<desmesh.size();pt++)
    {
        DstIm.at<Vec3b>(desmesh[pt].y,desmesh[pt].x)[0]=CCSIm.at<Vec3b>(srcmesh[pt].y,srcmesh[pt].x)[0];
        DstIm.at<Vec3b>(desmesh[pt].y,desmesh[pt].x)[1]=CCSIm.at<Vec3b>(srcmesh[pt].y,srcmesh[pt].x)[1];
        DstIm.at<Vec3b>(desmesh[pt].y,desmesh[pt].x)[2]=CCSIm.at<Vec3b>(srcmesh[pt].y,srcmesh[pt].x)[2];
    }
    srcmesh.clear();
    desmesh.clear();
}





static Vec6d mkHeadPose(const NrmParams &params,vector<Point2d> &Landmark1)
{
    const int nlandmark=6;
    Mat xFMark2D6LM(nlandmark,2,CV_64F);
    Mat xFMark2D6EPnP(nlandmark,1,CV_64FC2);
    for(size_t idx=0;idx<Landmark1.size();idx++)
    {
        xFMark2D6LM.at<double>(idx,0)=Landmark1[idx].x;
        xFMark2D6LM.at<double>(idx,1)=Landmark1[idx].y;
        xFMark2D6EPnP.at<Vec2d>(idx,0)[0]=Landmark1[idx].x;
        xFMark2D6EPnP.at<Vec2d>(idx,0)[1]=Landmark1[idx].y;
    }


    Vec3d rvec,tvec;
    Mat imagePoints,difference;
    solvePnP(params.facemodelEPnP,xFMark2D6EPnP, params.ProjMatrix,params.distCoef1,rvec,tvec,false,cv::SOLVEPNP_EPNP);
    solvePnP(params.facemodelLM,xFMark2D6LM,params.ProjMatrix,params.distCoef1, rvec,tvec,true,cv::SOLVEPNP_ITERATIVE);
    projectPoints(params.facemodelEPnP,rvec,tvec,params.ProjMatrix,params.distCoef1, imagePoints);

    absdiff(imagePoints,xFMark2D6EPnP,difference);
    Scalar meanval=mean(difference.reshape(1,6));
    //cout<<"project-error:"<<meanval.val[0]<<endl;
    Vec6d xHpose(rvec(0),rvec(1),rvec(2),tvec(0),tvec(1),tvec(2));
    return xHpose;
}



void DataNrmalization::mkTransformMat2(Matx33d&rotate, Vec3d&translatation)
{
    Matx33d vrRotate=mkVrRotate(rotate,translatation);
    _MR_=vrRotate.inv();
    double fnorm=norm(translatation,cv::NORM_L2);
    Matx33d ScaleR(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,params.vrDistance/fnorm);
    _MSR_=ScaleR*_MR_;
    _S2DIMxRot_=(params.VirIntrinsic1*_MSR_)*(params.invProjMatrix1);

#ifdef __WillTest_
    cout<<"vrRotate--------"<<endl;
    for(int row=0;row<3;row++)
        cout<<vrRotate(row,0)<<","<<vrRotate(row,1)<<","<<vrRotate(row,2)<<endl;
    cout<<"_MR_--------"<<endl;
    for(int row=0;row<3;row++)
        cout<<_MR_(row,0)<<","<<_MR_(row,1)<<","<<_MR_(row,2)<<endl;
    cout<<"_MSR_--------"<<endl;
    for(int row=0;row<3;row++)
        cout<<_MSR_(row,0)<<","<<_MSR_(row,1)<<","<<_MSR_(row,2)<<endl;
    cout<<"_S2DIMxRot_--------"<<endl;
    for(int row=0;row<3;row++)
        cout<<_S2DIMxRot_(row,0)<<","<<_S2DIMxRot_(row,1)<<","<<_S2DIMxRot_(row,2)<<endl;
#endif
}

Vec3d DataNrmalization::unNrmalOrient(Vec2d& angles,Vec3d& HCSOrient) const {
    Matx31d xDirection=RxSR2Orient(angles,_MR_);
    Matx31d xHCSDirection = _TransHpRotation_ * xDirection;

    HCSOrient = Vec3d(xHCSDirection(0,0),xHCSDirection(1,0),xHCSDirection(2,0));
    return Vec3d(xDirection(0,0),xDirection(1,0),xDirection(2,0));
}


void DataNrmalization::mkNormalImGz(const Mat&CCSIm,const vector<Point2f>&landmark)
{
    vector<Point2d> landmark1(6,Point2d(0,0));
    assert(landmark.size()==68 ||landmark.size()==6 ||landmark.size()==82);
    if(landmark.size()==68)
    {   int roipts[6]={36,39,42,45,48,54};
        for(int idx=0;idx<6;idx++)
            landmark1[idx]=Point2d(landmark[roipts[idx]].x,landmark[roipts[idx]].y);
    } else if(landmark.size()==82)
    {
        int roipts[6]={44, 48, 52, 56, 60, 66 };
        for(int idx=0;idx<6;idx++)
            landmark1[idx]=Point2d(landmark[roipts[idx]].x,landmark[roipts[idx]].y);

       /* for (size_t k = 0; k < landmark1.size(); k++) {
            cv::circle(CCSIm,  landmark1[k], 2,cv::Scalar(0,255,0),-1);
        }*/
//        cv::imshow("plt",CCSIm);
//        cv::imwrite("plt.bmp",CCSIm);
//        cv::waitKey(10);
//        cout<<endl;
    }
    else
    {
        for(int idx=0;idx<6;idx++)
            landmark1[idx]=Point2d(landmark[idx].x,landmark[idx].y);
    }


    Vec6d xHpose=mkHeadPose(params,landmark1);

    Mat rotation;
    Rodrigues(Vec3d(xHpose(0),xHpose(1),xHpose(2)), rotation);
    assert(rotation.type()==CV_64F);
    _HpRotation_=Matx33d(reinterpret_cast<double*>(rotation.data));
    transpose(rotation,rotation);
    _TransHpRotation_ = Matx33d(reinterpret_cast<double*>(rotation.data));

#ifdef __WillTest_
    for(int row=0;row<3;row++)
        {
            for(int col=0;col<3;col++)
                cout<<rotation.at<double>(row,col)<<",";
            cout<<endl;
        }
    for(int row=0;row<3;row++)
    {
        for(int col=0;col<3;col++)
            cout<<_HpRotation_(row,col)<<",";
        cout<<endl;
    }
#endif


    Mat Pts3D = params.facemodelLM*rotation;
    assert(Pts3D.type()==CV_64F);
    float xmean=0.0f,ymean=0.0f,zmean=0.0f;
    for(int idx=0;idx<6;idx++)
    {
        Pts3D.at<double>(idx,0)+=xHpose(3);
        Pts3D.at<double>(idx,1)+=xHpose(4);
        Pts3D.at<double>(idx,2)+=xHpose(5);
        xmean+= Pts3D.at<double>(idx,0);
        ymean+= Pts3D.at<double>(idx,1);
        zmean+= Pts3D.at<double>(idx,2);
    }

#ifdef __WillTest_
    for(int row=0;row<6;row++)
    cout<<Pts3D.at<double>(row,0)<<","<<Pts3D.at<double>(row,1)<<","<<Pts3D.at<double>(row,2)<<endl;
#endif


    {
        double xleft=0.5*(Pts3D.at<double>(0,0)+Pts3D.at<double>(1,0));
        double yleft=0.5*(Pts3D.at<double>(0,1)+Pts3D.at<double>(1,1));
        double zleft=0.5*(Pts3D.at<double>(0,2)+Pts3D.at<double>(1,2));

        double xright=0.5*(Pts3D.at<double>(2,0)+Pts3D.at<double>(3,0));
        double yright=0.5*(Pts3D.at<double>(2,1)+Pts3D.at<double>(3,1));
        double zright=0.5*(Pts3D.at<double>(2,2)+Pts3D.at<double>(3,2));

        _leftEyeCt_ =Point3d(xleft,yleft,zleft);
        _rightEyeCt_=Point3d(xright,yright,zright);
        _faceCt_=Point3d(xmean/6.0f,ymean/6.0f,zmean/6.0f);
    }



    Vec3d Translation(_faceCt_.x,_faceCt_.y,_faceCt_.z);
    mkTransformMat2(_HpRotation_, Translation);
    Mat NrmImage,NrmImage2;
//    TransformImage2(CCSIm,_S2DIMxRot_,params.VirIntrinsic1,NrmImage);
//    _tempNrmImage_=NrmImage.clone();
    warpPerspective( CCSIm, NrmImage2, _S2DIMxRot_, Size(448,448));
    cv::resize(NrmImage2,_tempNrmImage_,Size(112,112));
//    _tempNrmImage_=NrmImage2.clone();
//    imshow("NrmImage",NrmImage);
//    imshow("NrmImage2",NrmImage2);
//    waitKey(0);
}



void mkColorList(vector<cv::Scalar>&colList)
{
    colList.clear();
    cv::Scalar red=cv::Scalar(0,0,255);
    cv::Scalar green=cv::Scalar(0,255,0);
    cv::Scalar blue=cv::Scalar(255,0,0);
    cv::Scalar cyan=cv::Scalar(255,255,0);
    cv::Scalar orange=cv::Scalar(0,128,255);
    cv::Scalar magenta=cv::Scalar(255,0,255);
    colList.push_back(red);
    colList.push_back(green);
    colList.push_back(blue);
    colList.push_back(cyan);
    colList.push_back(orange);
    colList.push_back(magenta);
}



void DataNrmalization::wrtIntermediate(
        const string& rtpath,int frmidx)const
{
    ofstream faceCt(rtpath+"/faceCt_"+to_string(frmidx)+".txt",std::ios::out);
    faceCt<<_faceCt_.x<<","<<_faceCt_.y<<","<<_faceCt_.z<<endl;
    faceCt.close();
    ofstream leftEyeCt(rtpath+"/leftEyeCt_"+to_string(frmidx)+".txt",std::ios::out);
    leftEyeCt<<_leftEyeCt_.x<<","<<_leftEyeCt_.y<<","<<_leftEyeCt_.z<<endl;
    leftEyeCt.close();
    ofstream rightEyeCt(rtpath+"/rightEyeCt_"+to_string(frmidx)+".txt",std::ios::out);
    rightEyeCt<<_rightEyeCt_.x<<","<<_rightEyeCt_.y<<","<<_rightEyeCt_.z<<endl;
    rightEyeCt.close();

    ofstream MxR(rtpath+"/MxR_"+to_string(frmidx)+".txt",std::ios::out);
    MxR<<_MR_(0,0)<<","<<_MR_(0,1)<<","<<_MR_(0,2)<<endl;
    MxR<<_MR_(1,0)<<","<<_MR_(1,1)<<","<<_MR_(1,2)<<endl;
    MxR<<_MR_(2,0)<<","<<_MR_(2,1)<<","<<_MR_(2,2)<<endl;
    MxR.close();

    ofstream MxSR(rtpath+"/MxSR_"+to_string(frmidx)+".txt",std::ios::out);
    MxSR<<_MSR_(0,0)<<","<<_MSR_(0,1)<<","<<_MSR_(0,2)<<endl;
    MxSR<<_MSR_(1,0)<<","<<_MSR_(1,1)<<","<<_MSR_(1,2)<<endl;
    MxSR<<_MSR_(2,0)<<","<<_MSR_(2,1)<<","<<_MSR_(2,2)<<endl;
    MxSR.close();

    ofstream S2DIMxRot(rtpath+"/S2DIMxRot_"+to_string(frmidx)+".txt",std::ios::out);
    S2DIMxRot<<_S2DIMxRot_(0,0)<<","<<_S2DIMxRot_(0,1)<<","<<_S2DIMxRot_(0,2)<<endl;
    S2DIMxRot<<_S2DIMxRot_(1,0)<<","<<_S2DIMxRot_(1,1)<<","<<_S2DIMxRot_(1,2)<<endl;
    S2DIMxRot<<_S2DIMxRot_(2,0)<<","<<_S2DIMxRot_(2,1)<<","<<_S2DIMxRot_(2,2)<<endl;
    S2DIMxRot.close();

    ofstream HpRotation(rtpath+"/HpRotate_"+to_string(frmidx)+".txt",std::ios::out);
    HpRotation<<_HpRotation_(0,0)<<","<<_HpRotation_(0,1)<<","<<_HpRotation_(0,2)<<endl;
    HpRotation<<_HpRotation_(1,0)<<","<<_HpRotation_(1,1)<<","<<_HpRotation_(1,2)<<endl;
    HpRotation<<_HpRotation_(2,0)<<","<<_HpRotation_(2,1)<<","<<_HpRotation_(2,2)<<endl;
    HpRotation.close();
    imwrite(rtpath+"/NrmImage_"+to_string(frmidx)+".bmp",_tempNrmImage_);
//    Point3d _faceCt_;
//    Point3d _leftEyeCt_,_rightEyeCt_;
//    Mat _tempNrmImage_;
//    Matx33d _MR_,_MSR_,_S2DIMxRot_;
//    Matx33d _HpRotation_;
}




void readlandmarks(string& filename,
        vector<Point2f>&landmarks)
{
    ifstream file1(filename,std::ios::in);
    landmarks.clear();
    double xtemp,ytemp;
    while(!file1.eof())
    {
        xtemp=ytemp=-1;
        file1>>xtemp;
        cout<<file1.get()<<endl;
        file1>>ytemp;
        cout<<file1.get()<<endl;
        if(xtemp==-1 ||ytemp==-1)
            continue;
        landmarks.push_back(Point2f(float(xtemp),float(ytemp)));

    }
    file1.close();
    cout<<endl;
}






