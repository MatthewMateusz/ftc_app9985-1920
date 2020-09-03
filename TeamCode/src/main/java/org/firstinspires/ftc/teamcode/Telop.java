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

//add this vairable.  "CC" stands for controller coefficient and is used with mecanum wheels so the x and y sticks don't add up to more than 1 for a pair of motors (this would be bad since the motor value is limited to 1.  The motors moving in the opposing direction would not be limited
        double CC = .5;

        if (gamepad1.y)
            robot.servo_left_cont.setPower(.2);
        else
            robot.servo_left_cont.setPower(0);


        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)

//This code is for mecanum wheels mounted out the front and rear of robot (team 9985)
        leftFrontSpeed = (gamepad1.left_stick_y*(CC) - gamepad1.left_stick_x*(CC) + gamepad1.left_trigger*(CC) - gamepad1.right_trigger*(CC));
        rightFrontSpeed = (-gamepad1.left_stick_y*(CC) - gamepad1.left_stick_x*(CC) + gamepad1.left_trigger*(CC) - gamepad1.right_trigger*(CC));
        leftRearSpeed = (gamepad1.left_stick_y*(CC) + gamepad1.left_stick_x*(CC) + gamepad1.left_trigger*(CC) - gamepad1.right_trigger*(CC));
        rightRearSpeed = (-gamepad1.left_stick_y*(CC) +gamepad1.left_stick_x*(CC) + gamepad1.left_trigger*(CC) - gamepad1.right_trigger*(CC));

        robot.motor_frontLeft.setPower(leftFrontSpeed);
        robot.motor_frontRight.setPower(rightFrontSpeed);
        robot.motor_rearLeft.setPower(leftRearSpeed);
        robot.motor_rearRight.setPower(rightRearSpeed);

        telemetry.addData("LeftFront:",leftFrontSpeed);
        telemetry.addData("RightFront:",rightFrontSpeed);
        telemetry.addData("LeftRear:",leftRearSpeed);
        telemetry.addData("RightRear:",rightRearSpeed);

        telemetry.update();
//This code is for mecanum wheels mounted out the sides of robot (team 11283)



        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        //if (gamepad1.left_stick_y < 0) {
           // leftFrontSpeed = (gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);
            //rightFrontSpeed = (-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);
            //leftRearSpeed = (gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger);
            //rightRearSpeed = (-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger);

            //robot.servo_frontLeft.setPosition(.5);
            //robot.servo_frontRight.setPosition(.5);
            //robot.servo_rearRight.setPosition(.5);
            //robot.servo_rearLeft.setPosition(.5);
        }
        //else if (gamepad1.left_stick_y > 0) {
            //leftFrontSpeed = (gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);
            //rightFrontSpeed = (-gamepad1.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger);
            //leftRearSpeed = (gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger);
            //rightRearSpeed = (-gamepad1.left_stick_y - gamepad1.left_trigger + gamepad1.right_trigger);

            //robot.servo_frontLeft.setPosition(.5);
            //robot.servo_frontRight.setPosition(.5);
            //robot.servo_rearRight.setPosition(.5);
            //robot.servo_rearLeft.setPosition(.5);
        //}
        //else if (gamepad1.left_stick_x > 0) {
            //leftFrontSpeed = (-gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
            //rightFrontSpeed = (-gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
            //leftRearSpeed = (gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
            //rightRearSpeed = (gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);

        //}

        //else if (gamepad1.left_stick_x < 0) {
          //  leftFrontSpeed = (-gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
            //rightFrontSpeed = (-gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
            //leftRearSpeed = (gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);
            //rightRearSpeed = (gamepad1.left_stick_x + gamepad1.left_trigger - gamepad1.right_trigger);

        //}
        //else {
            //leftFrontSpeed = (gamepad1.left_trigger - gamepad1.right_trigger);
            //rightFrontSpeed = (gamepad1.left_trigger - gamepad1.right_trigger);
            //leftRearSpeed = (gamepad1.left_trigger - gamepad1.right_trigger);
            //rightRearSpeed = (gamepad1.left_trigger - gamepad1.right_trigger);

            //robot.servo_frontLeft.setPosition(1);
            //robot.servo_frontRight.setPosition(0);
            //robot.servo_rearRight.setPosition(0);
            //robot.servo_rearLeft.setPosition(1);
        //}






        //Servo position 1 is left and servo position right is 0 and servo straight is 0.5
        //if (gamepad1.x){
        //  robot.leftRearServo.setPosition(0);
        //}   else if (gamepad1.y) {
        //  robot.leftRearServo.setPosition(0.1);

        //}
        //else {

        //robot.motor_frontLeft.setPower(leftFrontSpeed);
        //robot.motor_frontRight.setPower(rightFrontSpeed);
        //robot.motor_rearLeft.setPower(leftRearSpeed);
        //robot.motor_rearRight.setPower(rightRearSpeed);

        /*robot.servo_frontLeft.setPosition(-.5*gamepad1.right_stick_x+.5);
        robot.servo_rearRight.setPosition(-.5*gamepad1.right_stick_x+.5);
        robot.servo_rearLeft.setPosition(-.5*gamepad1.right_stick_x+.5);
        robot.servo_frontRight.setPosition(-.5*gamepad1.right_stick_x+.5);*/

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
        //servoOffsetH = Range.clip(servoOffsetH, -0.5, 0.5);
        //servoOffsetV = Range.clip(servoOffsetV, -.5, 0.5);

        //robot.grabberVertServo.setPosition(robot.steeringstriaght - servoOffset);
//}

        // Send telemetry message to signify robot running;
        // telemetry.addData("vert Servo",  "position = %.2f",robot.steeringstriaght + servoOffsetV);
        //telemetry.addData("Servo Offset H","Offset H = %.2f", servoOffsetH);
        //telemetry.addData("Servo Offset V","Offset V = %.2f", servoOffsetV);

        //telemetry.addData("left",  "%.2f", left);
        //telemetry.update();

    //}

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}