package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Test Autonomous", group="INDEV")
public class TestAutonomous extends Automation {

    public void instruction() throws InterruptedException {
        while (opModeIsActive()) {
            Orientation angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            telemetry.addData("IMU", "INFO");
            telemetry.addData("Gyro X", angles.firstAngle);
            telemetry.addData("Gyro Y", angles.secondAngle);
            telemetry.addData("Gyro Z", angles.thirdAngle);
        }
    }
}
