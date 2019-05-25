#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "total_flow.h"
#include "bridge/result2u3d.pb.h"
#include "Util.h"
#include <vector>
#include <jni.h>
#define LOGD(...) ((void)__android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__))



extern "C" {



TotalFlow *totalFlow = nullptr;
Result *result = nullptr;
cv::Rect *bboxd;
cv::Rect *bboxf;
cv::Rect *bboxs;
cv::Rect *bboxc;
bool caliDone = false;
bool registerd = false;

JNIEXPORT jintArray JNICALL
Java_com_ut_sdk_DmsInfoGetter_FindFeatures2(JNIEnv *jniEnv, jobject obj,
                                                                jlong copyMat, jlong addrRgba,
                                                                jboolean regis, jboolean picture) {
    jintArray re1;
    jint *index2;
    if (totalFlow != nullptr) {
        totalFlow->Run(*(Mat *)copyMat, *result);
        std::string showFaceid = "name : ";
        std::string distration = "dis  : ";
        std::string fatigue = "fat  : ";
        std::string showSmoke = "smoke: ";
        std::string showCall = "call : ";
        std::string showAbnorm = "abnm : ";
        std::string showCalibrt = "calibrate : ";
        std::string faceid;

        if(!registerd && result->has_have_face()){
            registerd = totalFlow->FaceIDRun("untouch",HEAD_FORWARD::NORMAL);
        }

//        LOGE(" face id -%s- %d", result->name().c_str(), result->has_have_face());

        bool  doze,yawn,dis,call,smoke;
        doze = result->doze();
        yawn = result->yawn();
        call = result->phone();
        smoke = result->smoking();
        dis = false;
        if(caliDone && result->distraction() != DISTRACTION_TYPE::NORMAL)
            dis = true;

//        LOGE(" dis --  %d, yawn-- %d ; doze-- %d; phone -- %d ; smoke -- %d",dis, yawn,doze,call,smoke);

        re1 = jniEnv->NewIntArray(5);
        index2 = jniEnv->GetIntArrayElements(re1, NULL);

        index2[0] = dis;
        index2[1] = yawn;
        index2[2] = doze;
        index2[3] = call;
        index2[4] = smoke;

    }

    if (index2 != nullptr) {
        jniEnv->ReleaseIntArrayElements(re1, index2, 0);
    }


    return re1;
}

JNIEXPORT void JNICALL
Java_com_ut_sdk_DmsInfoGetter_FindFeatures(JNIEnv *jniEnv, jobject obj,
                                                               jlong addrGray, jint index) {
    if (totalFlow == nullptr) {
        totalFlow = new TotalFlow("/sdcard/untouch/model");

        result = new Result();
//        newMat = new Mat();
        bboxd = new cv::Rect();
        bboxf = new cv::Rect();
        bboxs = new cv::Rect();
        bboxc = new cv::Rect();
        LOGE("JNI abnormal -- init TotalFlow ----");
    }
}

JNIEXPORT jboolean JNICALL
Java_com_ut_sdk_DmsInfoGetter_Calibration(JNIEnv *jniEnv, jobject obj
                                           ){
    bool done = caliDone;
    if (totalFlow != nullptr && !done) {
//        LOGE(" Calibration");
        if (totalFlow->Calibration()) {
            caliDone = true;
            return JNI_TRUE;
        }
    }
    done = caliDone;
    if(done)
        return JNI_TRUE;
    else
        return JNI_FALSE;

//    return caliDone? JNI_TRUE:JNI_FALSE;
}

}



