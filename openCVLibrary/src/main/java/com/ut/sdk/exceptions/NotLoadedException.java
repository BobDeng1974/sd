package com.ut.sdk.exceptions;

import android.util.Log;

public class NotLoadedException extends Exception {
    @Override
    public void printStackTrace() {
        super.printStackTrace();
        Log.e("untouch :", "未调用 load()方法！");
    }

    @Override
    public String toString() {
        return "未调用 load()方法！";
    }
}
