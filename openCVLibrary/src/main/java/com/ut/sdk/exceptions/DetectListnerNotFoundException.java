package com.ut.sdk.exceptions;

import android.util.Log;

public class DetectListnerNotFoundException extends Exception {
    @Override
    public void printStackTrace() {
        super.printStackTrace();
        Log.e("untouch :", "未注册 DetectListner！");
    }

    @Override
    public String toString() {
        return "DetectListnerNotFoundException :未注册 DetectListner！";
    }
}
