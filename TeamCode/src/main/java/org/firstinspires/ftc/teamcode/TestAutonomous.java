package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name="testing", group="testing")
public class TestAutonomous extends Automation {

    @Override
    public void runOpMode() throws InterruptedException {
        setupHardware();
        initVuforia();
        initTfod();
        tfod.activate();
        waitForStart();
        distanceDrive(direction.forward, 2 * one_tile, 0.5, 2, true);

    }
}