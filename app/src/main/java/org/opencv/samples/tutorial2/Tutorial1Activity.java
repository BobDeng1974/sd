package org.opencv.samples.tutorial2;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.TextView;

import com.ut.sdk.DetectListner;
import com.ut.sdk.DmsInfoGetter;
import com.ut.sdk.InitializeListener;
import com.ut.sdk.R;
import com.ut.sdk.WarningInfo;
import com.ut.sdk.exceptions.CalibrationBeforeDetectException;
import com.ut.sdk.exceptions.DetectListnerNotFoundException;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class Tutorial1Activity extends Activity implements CvCameraViewListener2 {
    private static final String TAG = "OCVSample::Activity";

    private CameraBridgeViewBase mOpenCvCameraView;

    private Mat source;
    private Mat conerted;
    private TextView result;

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    mOpenCvCameraView.enableView();

                    //第1步，初始化算法，之后在opencv的摄像头获取数据回掉中操作检测工作
                        DmsInfoGetter.getInstance().initialize(new InitializeListener() {
                            @Override
                            public void onSuccess() {

                            }

                            @Override
                            public void onFail(String message) {
                            }
                        },mAppContext);

                    //第2步，设置检测的回调接口
                    DmsInfoGetter.getInstance().setDetectListner(new DetectListner() {
                        @Override
                        public void onDetect(WarningInfo warningInfo) {
                            Log.e(" warning info :", warningInfo.toString());
                            boolean isCall = warningInfo.isCall();
                            runOnUiThread(() -> result.setText(warningInfo.toString()));
                            //各种状态实时更新，检测到的各种状态为：
                            // isDistraction：分神；  yawn：打哈欠； doze：疲劳； call：打电话； smoke：吸烟
                        }
                    });
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public Tutorial1Activity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.tutorial1_surface_view);
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial1_activity_java_surface_view);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
        mOpenCvCameraView.setCameraIndex(1);
        mOpenCvCameraView.setMaxFrameSize(640,480);
        result = findViewById(R.id.result);
        findViewById(R.id.calibrate).setOnClickListener((view)->{
            try {
                DmsInfoGetter.getInstance().calibriation();//不进行校准的时候，分神检测不会触发。
            } catch (CalibrationBeforeDetectException e) {
                e.printStackTrace();
            }
        });
    }

    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        source = new Mat();
        conerted = new Mat();
    }

    public void onCameraViewStopped() {
        source.release();
        conerted.release();
    }

    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {

        source = inputFrame.rgba();
        if(source!= null && conerted!= null){
            Imgproc.cvtColor(source, conerted,Imgproc.COLOR_RGBA2RGB);
            try {
                DmsInfoGetter.getInstance().detect(conerted);

            } catch (DetectListnerNotFoundException e) {
                e.printStackTrace();
            }
        }

        return inputFrame.rgba();
    }
}
