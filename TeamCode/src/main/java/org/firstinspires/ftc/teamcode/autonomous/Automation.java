package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;

public abstract class Automation extends LinearOpMode {

    //Vars
    enum Direction {
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD
    }


    //NeveRest 40 Gearbox
    private static final int encoder_tick_per_revolution = 280;

    private static final double adjust = 1; //Adjust this variable if one centimeter distance travel is not one centimeter
    static final double encoder_cm = (encoder_tick_per_revolution / (7.62 * 3.14159265358979)) * adjust;


    static final double inch_in_cm = 2.54;
    static final double one_tile = 24 * inch_in_cm;
    static final long min_delay = 50;

    //Speed variables
    static final double speed_full  = 1;
    static final double speed_half  = 0.5;
    static final double speed_slow  = 0.3;
    static final double speed_still = 0.1;

    //timeout
    static final double timeout_short  = 3;
    static final double timeout_medium = 5;
    static final double timeout_long   = 10;

    private ElapsedTime runtime = new ElapsedTime();

    Hardware hardware = new Hardware();


    @Override
    public final void runOpMode() throws InterruptedException {
        //Do universal init stuff
        hardware.init(hardwareMap);

        //specific autonomous init code
        auto_init();

        telemetry.addData("Autonomous mode initialized", "Waiting for user start");
        telemetry.update();
        while(!isStarted() && !isStopRequested()) {
            idle();
        }

        //Run specific autonomous code
        instruction();
    }

    /*
    ** This method is where the user puts its instructions for autonomous mode.
    ** It is required since there is not point to an autonomous with no instructions
    */
    public abstract void instruction() throws InterruptedException;

    /*
    ** This method is where the user puts its custom init code.
    ** It also holds init code that is not required for each autonomous for example:
    ** Vuforia or neural networks
    */
    public void auto_init() {};


    /*
    **
    */
    void rotate(double angle, double power, double timeout, boolean brake) {

    }


    /*
    **
    */
    void driveDistance(double distance, double angle, double power, double timeout, boolean brake) {
        power = Range.clip(Math.abs(power), 0.0, 1.0);
    }


    /*
    **
    */
    void accurateDriveOLD(double vertical, double horizontal, double power, double timeout, boolean brake) {
        if (opModeIsActive()) {
            double speed = (Math.abs(power));

            double frontLeftSpeed  = 0;
            double frontRightSpeed = 0;
            double rearLeftSpeed   = 0;
            double rearRightSpeed  = 0;



        }
    }



    void encoderDrive(
            double frontLeftTicks,
            double frontRightTicks,
            double rearLeftTicks,
            double rearRightTicks,
            double power,
            double timeout,
            boolean brake
    ) {
        int frontLeftTarget;
        int frontRightTarget;
        int rearLeftTarget;
        int rearRightTarget;

        if (opModeIsActive()) {

            //Calc targets
            frontLeftTarget = hardware.motor_frontLeft.getCurrentPosition() + (int) frontLeftTicks;
            frontRightTarget = hardware.motor_frontRight.getCurrentPosition() + (int) frontLeftTicks;
            rearLeftTarget = hardware.motor_rearLeft.getCurrentPosition() + (int) frontLeftTicks;
            rearRightTarget = hardware.motor_rearRight.getCurrentPosition() + (int) frontLeftTicks;

            //Set targets
            hardware.motor_frontLeft.setTargetPosition(frontLeftTarget);
            hardware.motor_frontRight.setTargetPosition(frontRightTarget);
            hardware.motor_rearLeft.setTargetPosition(rearLeftTarget);
            hardware.motor_rearRight.setTargetPosition(rearRightTarget);

            setDriveMotorMode(RunMode.RUN_TO_POSITION);

            setDriveMotorSpeed(power);

            runtime.reset();

            while (
                    opModeIsActive() &&
                    notAtTarget(hardware.motor_frontLeft.getCurrentPosition(), frontLeftTarget, (int) frontLeftTicks) &&
                    notAtTarget(hardware.motor_frontRight.getCurrentPosition(), frontRightTarget, (int) frontRightTicks) &&
                    notAtTarget(hardware.motor_rearLeft.getCurrentPosition(), rearLeftTarget, (int) rearLeftTicks) &&
                    notAtTarget(hardware.motor_rearRight.getCurrentPosition(), rearRightTarget, (int) rearRightTicks) &&
                    runtime.seconds() < timeout
            ) {
                idle();
            }
            setDriveMotorSpeed(0);
        }
    }

    private void setDriveMotorMode(RunMode mode) {
        hardware.motor_frontLeft.setMode(mode);
        hardware.motor_frontRight.setMode(mode);
        hardware.motor_rearLeft.setMode(mode);
        hardware.motor_rearRight.setMode(mode);
    }

    private void setDriveMotorSpeed(double speed) {
        hardware.motor_frontLeft.setPower(speed);
        hardware.motor_frontRight.setPower(speed);
        hardware.motor_rearLeft.setPower(speed);
        hardware.motor_rearRight.setPower(speed);
    }

    private Boolean notAtTarget(
            int current,
            int destination,
            int direction
    ) {
        if (direction > 0) {
            return current <= destination;
        } else {
            return current >= destination;
        }
    }

    void runToTop() {
        hardware.servo_left_cont.setPower(0.1);
        while (
                opModeIsActive() &&
                !hardware.pressed(hardware.touch_Front)
        ) {
            idle();
        }
        hardware.servo_left_cont.setPower(-0.05);
    }

    void runToRear() {
        hardware.servo_left_cont.setPower(-0.1);
        while (
                opModeIsActive() &&
                        !hardware.pressed(hardware.touch_Rear)
        ) {
            idle();
        }
        hardware.servo_left_cont.setPower(-0.05);
    }

}
