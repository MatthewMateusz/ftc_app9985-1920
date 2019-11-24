package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="testing", group="testing")
public class TestAutonomous extends FunctionLibrary {

    @Override
    public void runOpMode() throws InterruptedException {

        setupHardware();
        waitForStart();

        vertical();
        encoderDriveDistance(speed_half, 2*one_tile, tMedium);
        encoderDriveDistance(speed_half, -2*one_tile, tMedium);
        horizontal();
        encoderDriveDistance(speed_half, 2*one_tile, tMedium);
        encoderDriveDistance(speed_half, -2*one_tile, tMedium);
    }
}