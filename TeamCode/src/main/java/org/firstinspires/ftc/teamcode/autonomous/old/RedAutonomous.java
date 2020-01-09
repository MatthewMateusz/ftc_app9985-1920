package org.firstinspires.ftc.teamcode.autonomous.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Autonomous", group="Normal")
public class RedAutonomous extends Automation {

    @Override
    public void runOpMode() throws InterruptedException {
        setupHardware();
        initVuforia();
        initTfod();
        tfod.activate();
        waitForStart();


        skyStone location = testBlockThing(250, 2);
        distanceDrive(direction.forward, 2 * one_tile, speed_half, 2, true);
        RotateArm(false, 2.5);

        //Phase A
        switch(location) {
            case left:
                distanceDrive(direction.forward, 0.4 * one_tile, speed_half, 1, true);
                setGrabber(grabber.left, grabState.close);
                break;

            default:
            case middle:
                distanceDrive(direction.forward, 0.4* one_tile, speed_half, 1, true);
                setGrabber(grabber.right, grabState.close);
                break;

            case right:
                distanceDrive(direction.right, 0.4 * one_tile, speed_half, 1, true);
                distanceDrive(direction.forward, 0.4*one_tile, speed_half, 1, true);
                setGrabber(grabber.right, grabState.close);
                sleep(300);
                break;

        }

        //Phase B
        distanceDrive(direction.backward, 0.4 * one_tile, speed_half, 1.5, true);
        RotateArm(true, 2.5);
        sensorDrive(direction.right, speed_half, 10, hardware.sensorDistanceUp, 2.5, false);
        distanceDrive(direction.right, 4 * one_tile, speed_half, 5, true);
        RotateArm(false, 2.5);
        liftArm(true, 1);
        distanceDrive(direction.forward, 0.5 * one_tile, speed_half, 1, true);
        setGrabber(grabber.all, grabState.open);
        distanceDrive(direction.backward, 0.5 * one_tile, speed_half, 2, true);


    }
}