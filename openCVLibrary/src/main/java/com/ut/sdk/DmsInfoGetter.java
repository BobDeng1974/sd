package com.ut.sdk;

import android.content.Context;
import android.util.Log;

import com.ut.sdk.exceptions.CalibrationBeforeDetectException;
import com.ut.sdk.exceptions.DetectBeforeInitializedException;
import com.ut.sdk.exceptions.DetectListnerNotFoundException;
import com.ut.sdk.exceptions.NotLoadedException;
import org.opencv.core.Mat;

import java.io.File;
import java.security.acl.LastOwnerException;

public class DmsInfoGetter {
    private boolean initialed = false;
    private boolean detected = false;
    private InitializeListener initializeListener;
    private DetectListner detectListner;

    private DmsInfoGetter(){

    }

    static class Holder{
        protected static DmsInfoGetter instance = new DmsInfoGetter();
    }

    public static DmsInfoGetter getInstance(){
        return Holder.instance;
    }

    protected static native int[] FindFeatures2(long add,long add2,boolean save, boolean arg);
    protected static native void FindFeatures(long add,int arg);
    protected static native boolean Calibration();


    public  void initialize(final InitializeListener listener, final Context context) {
        new Thread(new Runnable() {
            @Override
            public void run() {
                System.loadLibrary("native-lib");
                File f = new File("/sdcard/untouch/temp");
                if(!f.exists())
                    f.mkdirs();

                Utils.addModeles(context);

                FindFeatures(0L,0);
                if(initializeListener == null)
                    initializeListener = listener;
                if(initializeListener!= null)
                    initializeListener.onSuccess();
                initialed = true;
            }
        }).start();
    }

    public void setDetectListner(DetectListner detectListner) {
        this.detectListner = detectListner;
    }

    public void detect(Mat frame) throws DetectListnerNotFoundException{
        if(!initialed){
            Log.e("untouch :", new DetectBeforeInitializedException().toString());
            return;
        }
        if(detectListner!= null){
            int [] result = FindFeatures2(frame.getNativeObjAddr(),0L,false,false);
            detectListner.onDetect(new WarningInfo(result));
            detected = true;
        }else
            throw new DetectListnerNotFoundException();
    }

    public boolean calibriation() throws CalibrationBeforeDetectException {
        if(!detected)
            throw new CalibrationBeforeDetectException();
        return Calibration();
    }

}
