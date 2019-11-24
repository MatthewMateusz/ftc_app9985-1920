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
    final double    servoSpeedH   = 0.004;
    final double    servoSpeedV   = 0.004;
    final double    latchUp       = 0.5;
    final double    latchDown     = 0.25;

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
        double speedFactor;
        double liftSpeed = 0;
        double liftSpeedArm = 0;

        if (gamepad1.right_bumper)
            speedFactor = 0.3;
        else
            speedFactor = 1;


        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        if (gamepad1.right_stick_x <-.5) {
            leftFrontSpeed = (-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger) * speedFactor;
            rightFrontSpeed = (-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger) * speedFactor;
            leftRearSpeed = (-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger) * speedFactor;
            rightRearSpeed = (-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger) * speedFactor;

            robot.servo_frontLeft.setPosition(.5);
            robot.servo_frontRight.setPosition(.5);
            robot.servo_rearRight.setPosition(.5);
            robot.servo_rearLeft.setPosition(.5);
        }
        else if (gamepad1.right_stick_x >.5) {
            leftFrontSpeed = -(-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger) * speedFactor;
            rightFrontSpeed = -(-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger) * speedFactor;
            leftRearSpeed = -(-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger) * speedFactor;
            rightRearSpeed = -(-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger) * speedFactor;

            robot.servo_frontLeft.setPosition(.5);
            robot.servo_frontRight.setPosition(.5);
            robot.servo_rearRight.setPosition(.5);
            robot.servo_rearLeft.setPosition(.5);
        }
        else {
            leftFrontSpeed = -(-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger) * speedFactor;
            rightFrontSpeed = (-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger) * speedFactor;
            leftRearSpeed = -(-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger) * speedFactor;
            rightRearSpeed = (-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger) * speedFactor;

            robot.servo_frontLeft.setPosition(1);
            robot.servo_frontRight.setPosition(0);
            robot.servo_rearRight.setPosition(0);
            robot.servo_rearLeft.setPosition(1);
        }




        //Servo position 1 is left and servo position right is 0 and servo straight is 0.5
        //if (gamepad1.x){
        //  robot.leftRearServo.setPosition(0);
        //}   else if (gamepad1.y) {
        //  robot.leftRearServo.setPosition(0.1);

        //}
        //else {

        robot.motor_frontLeft.setPower(leftFrontSpeed);
        robot.motor_frontRight.setPower(rightFrontSpeed);
        robot.motor_rearLeft.setPower(leftRearSpeed);
        robot.motor_rearRight.setPower(rightRearSpeed);

        //robot.servo_frontLeft.setPosition(-.5*gamepad1.right_stick_x+.5);
        //robot.servo_rearRight.setPosition(-.5*gamepad1.right_stick_x+.5);
        //robot.servo_rearLeft.setPosition(-.5*gamepad1.right_stick_x+.5);
        //robot.servo_frontRight.setPosition(-.5*gamepad1.right_stick_x+.5);

        if  (gamepad2.dpad_up){
            robot.armMotorLift.setPower(1);}
        else if (!robot.pressed(robot.armLimitLiftDown) && gamepad2.dpad_down)
        {robot.armMotorLift.setPower(-.5);}
        else
        {robot.armMotorLift.setPower(0);}


        if (gamepad2.y && !robot.pressed(robot.armLimitRotateDown)) {
            robot.armMotorRotate.setPower(.5);
        }
        else if (gamepad2.a && !robot.pressed(robot.armLimitRotateUp)) {
            robot.armMotorRotate.setPower(-.5);
        }
        else {
            robot.armMotorRotate.setPower(0);
        }

        if (gamepad2.left_stick_y < 0 ) {
            robot.servo_GrabberLeft.setPosition(1);
           }

        else if (gamepad2.left_stick_y > 0) {
            robot.servo_GrabberLeft.setPosition(.2);
        }

        if (gamepad2.right_stick_y < 0 ) {
            robot.servo_GrabberRight.setPosition(1);
        }

        else if (gamepad2.right_stick_y > 0) {
            robot.servo_GrabberRight.setPosition(.2);
        }

        //Moves front servos
        //if (gamepad2.dpad_left)
        //  servoOffsetH += servoSpeedH;
        //else if (gamepad2.dpad_right)
        //  servoOffsetH -= servoSpeedH;
        //if (gamepad2.dpad_up)
        //  servoOffsetV += servoSpeedV;
        //else if (gamepad2.dpad_down)
        //servoOffsetV -= servoSpeedV;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        servoOffsetH = Range.clip(servoOffsetH, -0.5, 0.5);
        servoOffsetV = Range.clip(servoOffsetV, -.5, 0.5);

        //robot.grabberVertServo.setPosition(robot.steeringstriaght - servoOffset);
//}

        // Send telemetry message to signify robot running;
        // telemetry.addData("vert Servo",  "position = %.2f",robot.steeringstriaght + servoOffsetV);
        telemetry.addData("Servo Offset H","Offset H = %.2f", servoOffsetH);
        telemetry.addData("Servo Offset V","Offset V = %.2f", servoOffsetV);
        telemetry.addData("Distance Up (Inch)", "Distance Up = %.2f", robot.sensor_color_dist_up.getDistance(DistanceUnit.INCH));
        //telemetry.addData("left",  "%.2f", left);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
