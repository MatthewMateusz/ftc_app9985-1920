package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Right of Bridge", group="Bridges")
public class RightBridge extends Automation {

    @Override
    public void runOpMode() throws InterruptedException {
        setupHardware();
        waitForStart();

        sensorDrive(direction.left, speed_half, 10, hardware.sensorDistanceUp, 2.5, false);
    }
}
