package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Le Red", group="The Real Deal")
public class RedAutonomous extends FunctionLibrary {

    @Override
    public void runOpMode() throws InterruptedException {

        setupHardware();
        waitForStart();

        servo.vertical();
        motor.encoderDriveDistance(speed_half,0.25 * one_tile, tLong);
        servo.horizontal();
        motor.encoderDriveDistance(speed_half, -3 * one_tile, tLong);
        servo.vertical();
        motor.runToHit(hardware.armLimitRotateUp, speed_half, tLong);
        motor.runToUnHit(hardware.armLimitRotateUp, -speed_slow, tLong);
        rotateArmTime(1);
        liftArmTime(1);
        rotateArmTime(3);
        BackUP_stop(-speed_half, 5);




    }
}