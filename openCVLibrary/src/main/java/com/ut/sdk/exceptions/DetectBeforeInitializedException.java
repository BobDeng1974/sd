package com.ut.sdk.exceptions;

import android.util.Log;

public class DetectBeforeInitializedException extends Exception {
    @Override
    public void printStackTrace() {
        super.printStackTrace();
        Log.e("untouch :", "正在初始化中。。。");
    }

    @Override
    public String toString() {
        return "正在初始化中。。。";
    }
}
