package org.firstinspires.ftc.teamcode.autonomous;

public class PID {
    private double P;
    private double I;
    private double D;

    private double tolerance;

    private double minIn;
    private double maxIn;

    private double minOut;
    private double maxOut;

    private double prevError;
    private double prevI;


    public PID (double p, double i, double d) {
        P = p;
        I = i;
        D = d;
    }



}
