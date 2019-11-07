package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="testing", group="testing")
public class TestAutonomous extends FunctionLibrary {

    @Override
    public void runOpMode() throws InterruptedException {

        setupHardware();
        waitForStart();

        servo.vertical();
        motor.encoderDriveDistance(speed_half - 0.1,0.5 * one_tile, tLong);
        servo.horizontal();
        motor.encoderDriveDistance(speed_half - 0.1, -3 * one_tile, tLong);
        servo.vertical();
        motor.runToHit(hardware.armLimitRotateUp, speed_half - 0.1, tLong);
        motor.runToUnHit(hardware.armLimitRotateUp, -speed_slow, tLong);
        rotateArmTime(1);
        liftArmTime(1);
        rotateArmTime(3);
        BackUP_stop(speed_half, 5);




    }
}