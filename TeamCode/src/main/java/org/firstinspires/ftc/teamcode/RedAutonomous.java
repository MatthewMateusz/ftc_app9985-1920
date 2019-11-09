package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Pacifist", group="The Real Deal")
public class RedAutonomous extends FunctionLibrary {

    @Override
    public void runOpMode() throws InterruptedException {

        setupHardware();
        waitForStart();

        vertical();
        forwardOnTime(-0.4,2);
        horizontal();
        horizontalOnTime(-0.4, 7.9);
        vertical();
        runToHit(hardware.armLimitRotateUp, 0.4, tLong);
        runToUnHit(hardware.armLimitRotateUp, -speed_slow, tLong);
        rotateArmTime(1);
        liftArmTime(1);
        rotateArmTime(3);
        BackUP_stop(-speed_half, 5, 0.15);
        DrotateArmTime(3);
        horizontal();
        horizontalOnTime(speed_full, 1.25);





    }
}