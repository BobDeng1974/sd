#include <utility>
#include <fstream>
#include <iostream>
#include <algorithm>
#include"gaze_predict.h"
#include <utils/time/timer.h>

using namespace std;
using namespace cv;



void GazePredict::mkOutList()
{
    vector<string>defaultList(
    {"Input", "Out11","Out12","Out13",
    "Out21", "Out22", "Out23", "Out31","Out32",
    "Out41", "Out42", "Out51","Out52","Out53",
    "splitncnn_0", "wtOut11", "wtOut12",
    "wtOut21","wtOut22", "wtOut31","wtOut32",
    "Merged1","fcOut11", "fcOut12",
    "fcOut21", "fcOut22","fcOut23","fcOut31"});
    OutList.clear();
    for(auto nodename:defaultList)
        OutList.push_back(nodename);
}

void GazePredict::writeParams()const
{
    std::vector<int> typeIdxList;
    std::vector<std::string> typeList;
//    net.get_layer_idx_type(typeList, typeIdxList);
//    net.write_weights_bias();
    typeIdxList.clear();
    typeList.clear();
}

GazePredict::GazePredict(std::string param_path,std::string bin_path,std::string normalcfg_path)
        :_writeInputs_(false),_writeOutputs_(false),_writeParameter_(false),data_nrmal(normalcfg_path) {

    if(net.load_param(param_path.c_str()))
        std::cout << "load parameters failed. " << std::endl;
    if(net.load_model(bin_path.c_str()))
        std::cout <<"load model data failed. " << std::endl;
}


int GazePredict::writeInput(const ncnn::Mat &Input,const int frmidx) const
{
    string outpath=OutDtPath+string("/Input_")+to_string(frmidx)+string("_c0.txt");
    string outpath1=OutDtPath+string("/Input_")+to_string(frmidx)+string("_c1.txt");
    string outpath2=OutDtPath+string("/Input_")+to_string(frmidx)+string("_c2.txt");

    FILE* pFirst=fopen(outpath.c_str(),"wt");
    FILE* pSecond=fopen(outpath1.c_str(),"wt");
    FILE* pThird=fopen(outpath2.c_str(),"wt");

    const float* ptr0=Input.channel(0);
    const float* ptr1=Input.channel(1);
    const float* ptr2=Input.channel(2);

    for(int row=0;row<Input.h;row++)
    {
        for(int col=0;col<Input.w;col++,ptr0++,ptr1++,ptr2++)
        {
            fprintf(pFirst,"%.4f\t",*ptr0);
            fprintf(pSecond,"%.4f\t",*ptr1);
            fprintf(pThird,"%.4f\t",*ptr2);
        }
        fprintf(pFirst,"\n");
        fprintf(pSecond,"\n");
        fprintf(pThird,"\n");
    }
    fclose(pFirst);
    fclose(pSecond);
    fclose(pThird);
//np.savetxt("/home/public/nfs132_0/channel_x0.txt",tmpdata[0][0],fmt='%.4f')
//np.savetxt("/home/public/nfs132_0/channel_x1.txt", tmpdata[0][1],fmt='%.4f')
//np.savetxt("/home/public/nfs132_0/channel_x2.txt", tmpdata[0][2],fmt='%.4f')
    return 0;
}

int GazePredict::writeoutputs(const ncnn::Mat & output,const string &outname,const int frmidx) const
{
    const float* data=(const float*)(output.data);
    string outfile=OutDtPath+string("/x")+to_string(frmidx)+"_"+outname+".txt";
    FILE* pFile=fopen(outfile.c_str(),"wt");
    int len=output.w*output.h*output.c;
    for(int u=0;u<len;u++)
        fprintf(pFile,"%.10f\t",data[u]);
    fclose(pFile);
    return 0;
}


void GazePredict::mkInputData1(const cv::Mat &bgr, ncnn::Mat&Input) const
{
    assert(!bgr.empty());
    assert(bgr.cols==inputwidth);
    assert(bgr.rows==inputheight);
    assert(bgr.channels()==3);

    //ncnn::Mat Input(inputwidth, inputheight, 3, 4u);
    Input.create(inputwidth, inputheight, 3, 4u);

    if (Input.empty())
        return;

    cv::Mat mean(bgr.channels(),1,CV_64FC1);
    cv::Mat stddev(bgr.channels(),1,CV_64FC1);
    cv::meanStdDev(bgr,mean,stddev);
    if(false)
    {
        std::cout<<mean.at<double>(0)<<","<<mean.at<double>(1)<<","<<mean.at<double>(2)<<std::endl;
        std::cout<<stddev.at<double>(0)<<","<<stddev.at<double>(1)<<","<<stddev.at<double>(2)<<std::endl;
    }


    float mean0=float(mean.at<double>(0));
    float mean1=float(mean.at<double>(1));
    float mean2=float(mean.at<double>(2));

    float stddev0=float(stddev.at<double>(0));
    float stddev1=float(stddev.at<double>(1));
    float stddev2=float(stddev.at<double>(2));


    float* ptr0 = Input.channel(0);
    float* ptr1 = Input.channel(1);
    float* ptr2 = Input.channel(2);
    uchar* pBGR=bgr.data;
    for(int idx=0;idx<(112*112);idx++,pBGR+=3,ptr0++,ptr1++,ptr2++)
    {
        *ptr0=(pBGR[0]-mean0)/stddev0;
        *ptr1=(pBGR[1]-mean1)/stddev1;
        *ptr2=(pBGR[2]-mean2)/stddev2;
    }
//    cv::imshow("bgr",bgr);
//    cv::waitKey(0);
    return;
}

void GazePredict::mkInputData2(const cv::Mat&bgr,ncnn::Mat & Input) const
{
    assert(!bgr.empty());
    assert(bgr.cols==inputwidth);
    assert(bgr.rows==inputheight);

    //cv::transpose(bgr,bgr);
    //Input.create(inputwidth, inputheight,channels,4u);
    //ncnn::Mat temp = ncnn::Mat::from_pixels(bgr.data,ncnn::Mat::PIXEL_GRAY,inputwidth, inputheight);
    //memcpy(Input.data, temp.data, temp.total() * temp.elemsize);
    Input = ncnn::Mat::from_pixels(bgr.data,ncnn::Mat::PIXEL_GRAY,inputwidth, inputheight);


    /*cv::Mat mean(bgr.channels(),1,CV_64FC1);
    cv::Mat stddev(bgr.channels(),1,CV_64FC1);
    cv::meanStdDev(bgr,mean,stddev);

    double* pmdata=(double*)(mean.data);
    double* psdata=(double*)(stddev.data);
    float meanvals[3]={float(pmdata[0]),float(pmdata[1]),float(pmdata[2])};
    float stdvals[3]={float(psdata[0]),float(psdata[1]),float(psdata[2])};
    float stdvals2[3]={float(1.0/psdata[0]),float(1.0/psdata[1]),float(1.0/psdata[2])};
    Input.substract_mean_normalize(meanvals,stdvals2);*/

   return;
}


void GazePredict::Predict(const cv::Mat& gray,const std::vector<cv::Point2f>& shapes,
        cv::Vec3d& gaze_direction,cv::Vec3d& HCSOrient)
{
    Timer set_input_time("set input time : ");
    cv::Mat NrmImage;
    //NrmImage=cv::imread("../input.png");
    data_nrmal.mkNormalImGz(gray,shapes);
    data_nrmal.QueryNrmImage(NrmImage);


    ncnn::Mat net_output;
    ncnn::Mat Input;
    Timer input_data_timer("input data timer : ");
    mkInputData2(NrmImage,Input);
//    std::cout << input_data_timer << std::endl;
    //cv::imwrite("../input.png",NrmImage);
//    std::cout << set_input_time << std::endl;
    ncnn::Mat Output;
    Timer forward_time("forward-time : ");
    ncnn::Extractor ex = net.create_extractor();
    ex.set_num_threads(1);
    ex.input("data", Input);
    ex.extract("gaze2", Output);
//    std::cout << forward_time << std::endl;
    //ex.input("Input", Input);
    //ex.extract("OutPut", Output);

    Timer set_output_timer("output time : ");
    net_output.create(Output.w,Output.h,Output.c,4u);
    memcpy(net_output.data,Output.data,Output.total()*Output.elemsize);
    cv::Vec2d angles(double(net_output.row(0)[0]),double(net_output.row(0)[1]));
//    std::cout<<"angles="<<angles<<std::endl;

    gaze_direction = data_nrmal.unNrmalOrient(angles,HCSOrient);
//    std::cout << set_output_timer << std::endl;
}

void GazePredict::DebugDraw(cv::Mat& image,cv::Vec3d& gaze_direction) {
    data_nrmal.pltVisualDirect(image,gaze_direction);
}

cv::Point3d GazePredict::GetLeftEyeCt() {
    return data_nrmal.GetLeftEyeCt();
}

cv::Point3d GazePredict::GetRightEyeCt() {
    return data_nrmal.GetRightEyeCt();
}



GazePredict::~GazePredict()
{
    net.clear();
    OutList.clear();
}

