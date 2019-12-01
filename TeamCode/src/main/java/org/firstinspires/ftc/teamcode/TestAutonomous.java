package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="testing", group="testing")
public class TestAutonomous extends FunctionLibrary {

    @Override
    public void runOpMode() throws InterruptedException {
    initVuforia();
    initTfod();
    tfod.activate();

    waitForStart();

    encoderDriveDistance(speed_half, 1 * one_tile, 2);
    testBlockThing(500, 2);

    }
}