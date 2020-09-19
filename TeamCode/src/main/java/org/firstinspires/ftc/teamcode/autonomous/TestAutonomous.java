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
        rotate(180, 0.5, 5, true);
        sleep(1000);
        //driveIMUDistance(100, 0, 0.5, 10, true);
    }
}
