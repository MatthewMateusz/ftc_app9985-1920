package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Left of Bridge", group="testing")
public class RightBridge extends FunctionLibrary {

    @Override
    public void runOpMode() throws InterruptedException {
        try {

            setupHardware();
            waitForStart();
            horizontal();
            driveSensorDistance(speed_half, hardware.sensorDistanceUp, 10, 3);

        } catch (Exception e) {
            STOP();
            throw e;
        }
    }
}