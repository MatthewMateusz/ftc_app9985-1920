package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="testing", group="testing")
public class TestAutonomous extends FunctionLibrary {

    @Override
    public void runOpMode() throws InterruptedException {
        setupHardware();
        initVuforia();
        initTfod();
        tfod.activate();
        waitForStart();

        vertical();
        skyStone location = testBlockThing(250, 2);
        encoderDriveDistance(speed_half, 1 * one_tile, 2);
        RotateArm(false, 2.5);

        //Phase A
        switch (location) {
            case left:
                encoderDriveDistance(speed_half, 0.3 * one_tile, 2);
                setGrabber(grabber.left, grabState.close);
                break;

            case middle:
                encoderDriveDistance(speed_half, 0.3 * one_tile, 2);
                setGrabber(grabber.right, grabState.close);
                break;

            case right:
                horizontal();
                encoderDriveDistance(speed_half, -0.25 * one_tile, 2);
                vertical();
                encoderDriveDistance(speed_half, 0.3 * one_tile, 2);
                setGrabber(grabber.right, grabState.close);
                break;

            default:
                break;
        }

        //Phase B
        encoderDriveDistance(speed_half, -0.3 * one_tile, 2); // Move back
        RotateArm(true, 2.5); //Rotate arm back for driving under bridge
        horizontal(); // servo movement
        driveSensorDistance(-speed_half, hardware.sensorDistanceUp, 10, 2.5); // drive until the distance sensor up reads less than 10cm
        encoderDriveDistance(speed_half, -4 * one_tile, 5); //Move out side of bridge
        vertical(); // Servo movement
        RotateArm(false, 2.5); // Rotate arm for block placing position.
        liftArm(true, 0.75); //Lift arm so to not clip the building site
        encoderDriveDistance(speed_half, 0.25 * one_tile, 1); // Move forward
        setGrabber(grabber.all, grabState.open);
        encoderDriveDistance(speed_half, -0.5 * one_tile, 2); // Drive away from the building site
        liftArm(false, 0.75); //Lift arm down
        RotateArm(true, 2.5); //Rotate arm back for driving under bridge
        horizontal(); //Horizontal movement time(tm)
        driveSensorDistance(speed_half, hardware.sensorDistanceUp, 10, 5); // Drive under the bridge

    }
}