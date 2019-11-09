package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Kamikaze", group="The Real Deal")
public class BlueKillAutonomous extends FunctionLibrary {

    @Override
    public void runOpMode() throws InterruptedException {

        setupHardware();
        waitForStart();

        vertical();
        forwardOnTime(-0.4,2);
        horizontal();
        horizontalOnTime(0.4, 7.9);
        vertical();
        runToHit(hardware.armLimitRotateUp, 0.4, tLong);
        runToUnHit(hardware.armLimitRotateUp, -speed_slow, tLong);
        rotateArmTime(1);
        liftArmTime(1);
        rotateArmTime(3);
        BackUP_stop(speed_half, 5, 0.85);
        DrotateArmTime(3);
        horizontal();
        horizontalOnTime(-speed_full, 2.5);





    }
}