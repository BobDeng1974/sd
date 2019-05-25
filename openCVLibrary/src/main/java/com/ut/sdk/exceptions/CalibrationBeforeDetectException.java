package com.ut.sdk.exceptions;

import android.util.Log;

public class CalibrationBeforeDetectException extends Exception {
    @Override
    public void printStackTrace() {
        super.printStackTrace();
        Log.e("untouch :", "请在调用过detect之后进行calibration");
    }

    @Override
    public String toString() {
        return "请在调用过detect之后进行calibration";
    }
}
