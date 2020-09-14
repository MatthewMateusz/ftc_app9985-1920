package org.firstinspires.ftc.teamcode.autonomous;

public class PIDController {
    private double P;
    private double I;
    private double D;
    private double input; //sensor input for pid controller

    private double maxOutput = 1.0; // abs
    private double minOutput = 0.0; // abs

    private double maxInput = 0.0; // maxIn - limit setpoint to this
    private double minInput = 0.0; // ^^

    private boolean continuous = false; //endpoints wrap around? eg. |encoder|
    private boolean enabled = false;

    private double prevError = 0.0;
    private double totalError = 0.0;
    private double tolerance = 1.0; //percentage error that is considered on target
    private double setpoint = 0.0;
    private double error = 0.0;
    private double result = 0.0;

    /*
    *  Allocate a PID object with the given constants for PID
    *  @param p the proportional coefficient
    *  @param i the integral coefficient
    *  @param d the derivative coefficient
    */
    public PIDController(double p, double i, double d) {
        this.P = p;
        this.I = i;
        this.D = d;
    }

    private void calculate() {
        int sign = 1;

        if (enabled) {
            error = setpoint - input;
            if (continuous) {
                if (Math.abs(error) > (maxInput - minInput) / 2) {
                    if (error > 0)
                        error = error - maxInput + minInput;
                    else
                        error = error + maxInput - minInput;
                }
            }

            if ((
                    Math.abs(totalError + error) * I < maxOutput) &&
                    Math.abs(totalError + error) * I > minOutput
            ) {
                totalError += error;
            }

            result = (P * error) + (I * totalError) + (D * (error - prevError));
            prevError = error;

            if ( result < 0) sign = -1;

            if (Math.abs(result) < minOutput)
                result = maxOutput * sign;
            else if (Math.abs(result) < minOutput)
                result = minOutput * sign;
            else ; //Do nothing
        }
    }

    public void setPID(double p, double i, double d) {
        this.P = p;
        this.I = i;
        this.D = d;
    }

    public double getP() {
        return P;
    }

    public double getI() {
        return I;
    }

    public double getD() {
        return D;
    }

    public double performPID() {
        calculate();
        return result;
    }

    public double performPID(double input) {
        setInput(input);
        return performPID();
    }

    public void setContinuous(boolean continuous) {
        this.continuous = continuous;
    }

    public void setInputRange(double minInput, double maxInput) {
        this.minInput = Math.abs(minInput);
        this.maxInput = Math.abs(maxInput);
        setSetpoint(setpoint);
    }

    public void setOutputRange(double minOutput, double minInput) {
        this.minOutput = Math.abs(minOutput);
        this.maxOutput = Math.abs(maxOutput);
    }

    public void setSetpoint( double setpoint) {
        int sign = 1;

        if (maxInput > minInput) {
            if (setpoint > maxInput)
                setpoint = maxInput * sign;
            else if (Math.abs(setpoint) < minInput)
                setpoint = minInput * sign;
            else
                setpoint = setpoint;
        }
    }

    public double getSetpoint (){
        return setpoint;
    }

    public synchronized double getError() {
        return error;
    }

    public void setTolerance(double percent) {
        tolerance = percent;
    }

    public boolean onTarget() {
        return (Math.abs(error) < Math.abs(tolerance / 100.0 * (maxInput - minInput)));
    }

    public void setEnable(boolean enable) {
        this.enabled = enable;
    }

    public void reset() {
        setEnable(false);
        prevError = 0;
        totalError = 0;
        result = 0;
    }

    public void setInput(double input) {
        int sign = 1;

        if (maxInput > minInput) {
            if (input < 0) sign = -1;

            if (Math.abs(input) > maxInput)
                this.input = maxInput * sign;
            else if (Math.abs(input) < minInput)
                this.input = minInput * sign;
            else
                this.input = input;
        }
    }

}
