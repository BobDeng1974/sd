#ifndef SINNCNN_NCNNPREDICT_H
#define SINNCNN_NCNNPREDICT_H
#include "net.h"
#include "mat.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "Nrmal.h"

class GazePredict
{
public:
    ~GazePredict();
    GazePredict(std::string param_path,std::string bin_path,std::string normal_config);
    explicit GazePredict(const std::string& path)
            :  GazePredict(path + "/gaze.param", path + "/gaze.bin", path + "/NormalCfg1.xml"){}

    /**
     *
     * @param bgr
     * @param shapes
     * @param gaze_direction [out] send to Unity3d to draw gaze on right image.
     * @param HCSOrient [out] send to Unity3d to draw gaze on left model.
     */
    void Predict(const cv::Mat& bgr,const std::vector<cv::Point2f>& shapes,cv::Vec3d& gaze_direction,cv::Vec3d& HCSOrient);

    void DebugDraw(cv::Mat& image,cv::Vec3d& gaze_direction);

    cv::Point3d GetLeftEyeCt();
    cv::Point3d GetRightEyeCt();


private:
    ncnn::Net net;
    std::string OutDtPath;
    std::string _inputnode_;
    std::string _outputnode_;
    bool _writeInputs_;
    bool _writeOutputs_;
    bool _writeParameter_;
    const int channels=1;
    const int inputwidth=112;
    const int inputheight=112;
    std::vector<std::string>OutList;

    DataNrmalization data_nrmal;

private:
    void mkOutList();
    void writeParams()const;
    void mkInputData1(const cv::Mat& bgr,ncnn::Mat &Input) const;
    void mkInputData2(const cv::Mat& bgr,ncnn::Mat &Input) const;
    int writeInput(const ncnn::Mat &Input,const int frmidx=1) const;
    int writeoutputs(const ncnn::Mat & output,const std::string &outname,const int frmidx=1) const;
};

#endif //SINNCNN_NCNNPREDICT_H

