package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.Automation;

@Autonomous(name="setup", group = "testing")
public class CalibrateHardware extends Automation {

    @Override
    public void runOpMode() throws InterruptedException {

        setupHardware();
        waitForStart();
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
