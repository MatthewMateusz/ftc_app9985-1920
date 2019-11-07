package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="setup", group = "testing")
public class CalibrateHardware extends FunctionLibrary {

    @Override
    public void runOpMode() throws InterruptedException {

        setupHardware();
        waitForStart();
        servo.horizontal();

        while (opModeIsActive()) {
            idle();
        }
    }
}
