package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Autonomous", group="INDEV")
public class TestAutonomous extends Automation {

    Thread moveServo = new Thread(new ToTop());

    public void instruction() throws InterruptedException {

        moveServo.start();
        //encoderDrive(10000, 10000, 10000, 10000, 0.1, 30, true);
        moveServo.join();

    }

    private class ToTop implements Runnable {

        @Override
        public void run() {
            runToTop();
            runToRear();
        }
    }
}
