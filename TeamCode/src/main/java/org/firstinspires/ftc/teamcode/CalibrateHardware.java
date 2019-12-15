package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="setup", group = "testing")
public class CalibrateHardware extends Automation {

    @Override
    public void runOpMode() throws InterruptedException {

        setupHardware();
        waitForStart();
        horizontal();
        initVuforia();
        initTfod();
        tfod.activate();

        while (opModeIsActive()) {
            idle();
            telemetry.addData("Opmode", opModeIsActive());
            telemetry.update();
        }
    }
}
