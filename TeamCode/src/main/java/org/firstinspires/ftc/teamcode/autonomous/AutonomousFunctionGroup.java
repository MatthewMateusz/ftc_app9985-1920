package org.firstinspires.ftc.teamcode.autonomous;

/**
 *  Used when different paths are available in autonomous
 */
public abstract class AutonomousFunctionGroup {

    private boolean functionComplete = false;
    private boolean started = false;

    public boolean isComplete() {
        return functionComplete;
    }

    public boolean isStarted() {
        return started;
    }

    public abstract void instructions();

    public void execute () {
        started = true;
        instructions();
    }

    public boolean canStart(AutonomousFunction function) {
        if (function.isComplete() && !function.isStarted()) {
            return true;
        } else {
            return false;
        }
    }

    public boolean canStart(AutonomousFunctionGroup function) {
        if (function.isComplete() && !function.isStarted()) {
            return true;
        } else {
            return false;
        }
    }
}
