package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
//@Disabled
public class Telop extends OpMode {
    /* Declare OpMode members. */
    Hardware robot       = new Hardware(); // use the class created to define a Pushbot's hardware

    double          servoOffsetH  = 0.0;                  // Servo mid position
    double          servoOffsetV  = 0.5;
    //final double    servoSpeedH   = 0.004;
    //final double    servoSpeedV   = 0.004;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "No one out pizzas the hut, good luck"); //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */


    @Override
    public void loop() {

        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftRearSpeed;
        double rightRearSpeed;





        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        if (gamepad1.left_stick_y < 0) {
            leftFrontSpeed = (gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);
            rightFrontSpeed = (-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);
            leftRearSpeed = (gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger);
            rightRearSpeed = (-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger);

        }
        else if (gamepad1.left_stick_y > 0) {
            leftFrontSpeed = (gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);
            rightFrontSpeed = (-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);
            leftRearSpeed = (gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger);
            rightRearSpeed = (-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger);

        }
        else if (gamepad1.left_stick_x > 0) {
            leftFrontSpeed = (-gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
            rightFrontSpeed = (-gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
            leftRearSpeed = (gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
            rightRearSpeed = (gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);

        }

        else if (gamepad1.left_stick_x < 0) {
            leftFrontSpeed = (-gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
            rightFrontSpeed = (-gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
            leftRearSpeed = (gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
            rightRearSpeed = (gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);

        }
        else {
            leftFrontSpeed = (gamepad1.left_trigger - gamepad1.right_trigger);
            rightFrontSpeed = (gamepad1.left_trigger - gamepad1.right_trigger);
            leftRearSpeed = (gamepad1.left_trigger - gamepad1.right_trigger);
            rightRearSpeed = (gamepad1.left_trigger - gamepad1.right_trigger);

        }

        if (gamepad1.y)
            robot.servo_left_cont.setPower(.2);
        else
            robot.servo_left_cont.setPower(0);


        robot.motor_frontLeft.setPower(leftFrontSpeed);
        robot.motor_frontRight.setPower(rightFrontSpeed);
        robot.motor_rearLeft.setPower(leftRearSpeed);
        robot.motor_rearRight.setPower(rightRearSpeed);

        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}