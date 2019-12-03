package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="testing", group="testing")
public class TestAutonomous extends FunctionLibrary {

    @Override
    public void runOpMode() throws InterruptedException {
    initVuforia();
    initTfod();
    tfod.activate();
    setupHardware();
    waitForStart();

    vertical();
    encoderDriveDistance(speed_half, 1 * one_tile, 2);
    //skyStone location = testBlockThing(500, 2);
    skyStone location = skyStone.right;
    RotateArm(false, 2.5);

    if (location.equals(skyStone.left)) {
        encoderDriveDistance(speed_half, 0.3 * one_tile, 2);
        setGrabber(grabber.left, grabState.close);
        encoderDriveDistance(speed_half, -0.3 * one_tile, 2);
        RotateArm(true, 2.5);

    } else if (location.equals(skyStone.middle)) {
        encoderDriveDistance(speed_half, 0.3 * one_tile, 2);
        setGrabber(grabber.right, grabState.close);
        encoderDriveDistance(speed_half, -0.3 * one_tile, 2);
        RotateArm(true, 2.5);

    } else if (location.equals(skyStone.right)) {
        horizontal();
        encoderDriveDistance(speed_half, -0.25 * one_tile, 2);
        vertical();
        encoderDriveDistance(speed_half, 0.3 * one_tile, 2);
        setGrabber(grabber.right, grabState.close);
        encoderDriveDistance(speed_half, -0.3 * one_tile, 2);
        RotateArm(true, 2.5);

    } else {

    }

    }
}