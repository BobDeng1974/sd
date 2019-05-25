package com.ut.sdk;

import android.util.Log;

public class WarningInfo {
    private boolean distraction = false;
    private boolean yawn = false;
    private boolean doze = false;
    private boolean call = false;
    private boolean smoke = false;

    protected WarningInfo(int [] data){
        distraction = data[0] == 1;
        yawn = data[1] == 1;
        doze = data[2] == 1;
        call = data[3] == 1;
        smoke = data[4] == 1;
        Log.e(" warning info : " , toString());
    }

    public boolean isDistraction() {
        return distraction;
    }

    public boolean isYawn() {
        return yawn;
    }

    public boolean isDoze() {
        return doze;
    }

    public boolean isCall() {
        return call;
    }

    public boolean isSmoke() {
        return smoke;
    }

    @Override
    public String toString() {
        return " dis: " + distraction + " | yawn: "+ yawn + " | doze: " + doze + " | phone: " + call + " | smoke: "+smoke;
    }
}
