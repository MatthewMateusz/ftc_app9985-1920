package org.firstinspires.ftc.teamcode.autonomous;

public abstract class AutonomousFunction {

    private boolean functionComplete = false;
    private boolean started = false;
    private boolean usesDrive = true;
    private boolean allowNext = false;


    public boolean isComplete() {
        return functionComplete;
    }

    public boolean isStarted() {
        return started;
    }

    public boolean useDrive() {
        return usesDrive;
    }

    public boolean allowNext() {
        return allowNext;
    }

    public void execute_start(){
        instructions();
    }

    public void execute(AutonomousFunction function) {
        if (canStart(function)) {
            reset();
            instructions();
        }
    }

    public void execute(AutonomousFunctionGroup function) {
        if (canStart(function)) {
            reset();
            instructions();
        }
    }

    public abstract void instructions();

    public boolean canStart(AutonomousFunction function) {
        if (!function.allowNext()) {
            if (function.isComplete() && !function.isStarted()) {
                return true;
            } else {
                return false;
            }
        } else {
            if (usesDrive) {
                if (function.useDrive()) {
                    return false;
                } else {
                    return true;
                }
            } else {
                return true;
            }
        }
    }

    public boolean canStart(AutonomousFunctionGroup function) {
        if (function.isComplete() && !function.isStarted()) {
            return true;
        } else {
            return false;
        }
    }

    public abstract boolean test();

    public void terminate() {
        stop();
        functionComplete = true;
        started = false;
    }

    public abstract void stop();

    public void reset() {
        functionComplete = false;
        started = false;
    }
}
