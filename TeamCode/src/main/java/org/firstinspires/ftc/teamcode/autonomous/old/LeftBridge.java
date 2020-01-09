package org.firstinspires.ftc.teamcode.autonomous.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Left of Bridge", group="Bridges")
public class LeftBridge extends Automation {

    @Override
    public void runOpMode() throws InterruptedException {
        setupHardware();
        waitForStart();

        sensorDrive(direction.right, speed_half, 10, hardware.sensorDistanceUp, 2.5, false);
    }
}
