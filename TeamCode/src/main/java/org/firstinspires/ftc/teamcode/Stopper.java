package org.firstinspires.ftc.teamcode;

import java.util.Timer;

public class Stopper extends java.util.TimerTask{
    private boolean canRun = false;
    private long stopAfter = 29500;
    private Timer time = null;

    public Stopper() {

    }

    public Stopper (long wait) {
        this.stopAfter = wait;
    }

    public void start() {
        /*time = new Timer();
        time.schedule(this, stopAfter);*/
        canRun = true;
        (time = new Timer()).schedule(this, stopAfter);
    }

    public void stop() {
        canRun = false;
        time.cancel();
    }

    public boolean canRun() {
        return canRun;
    }

    @Override
    public void run() {
        canRun = false;
    }
}
