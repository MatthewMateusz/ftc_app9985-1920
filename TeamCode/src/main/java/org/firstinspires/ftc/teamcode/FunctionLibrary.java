package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

abstract public class FunctionLibrary  extends LinearOpMode {

    //Variables ...






    enum motorOrientation {
        forward,
        reverse
    }

    motorOrientation frontLeft;
    motorOrientation frontRight;
    motorOrientation rearLeft;
    motorOrientation rearRight;


    //NeveRest 40 Gearbox
     static final int encoder_tick_per_revolution = 280;

    //3-inch wheel (7.62cm)
    static final double adjust = 4;
    static final double encoder_cm = (encoder_tick_per_revolution / (7.62 * 3.141592653589793)) * adjust;

    //distance variables
    static final double inch_to_cm = 2.54;
    static final double one_tile = 24 * inch_to_cm;
    static final long min_delay = 100;

    //speed variables
    public static final double speed_full = 1;
    public static final double speed_normal = 0.7;
    public static final double speed_half = 0.5;
    public static final double speed_slow = 0.3;
    public static final double speed_death = 0.1;

    //timeout
    public static final double tShort = 3;
    public static final double tMedium = 5;
    public static final double tLong = 10;

    private ElapsedTime runtime = new ElapsedTime();

    Hardware hardware = new Hardware();

    //Setups the hardware for the hardware
    void setupHardware() {
        hardware.init(hardwareMap);
    }

    //Waits until the start button is pressed on the phone
    public void waitForStart() {
        while (!isStarted() && !isStopRequested()) {
            idle();
        }
    }

    public void forwardOnTime(double speed, double time) {
        setDriveMotorMode_all(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();
        setDriveMotorPower(speed, -speed, speed, -speed);
        while (opModeIsActive() && runtime.seconds() <= time) {
            idle();
        }
        setDriveMotorPower_all(0);
        setDriveMotorMode_all(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(min_delay);
    }

    public void horizontalOnTime(double speed, double time) {
        setDriveMotorMode_all(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();
        setDriveMotorPower_all(speed);
        while (opModeIsActive() &&
                runtime.seconds() <= time) {
            idle();
        }
        setDriveMotorPower_all(0);
        setDriveMotorMode_all(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(min_delay);
    }


    public void rotateArmTime(double timeout) {
        hardware.armMotorRotate.setPower(-speed_half);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= timeout && !hardware.pressed(hardware.armLimitRotateUp)) {
            idle();
        }
        hardware.armMotorRotate.setPower(0);
        sleep(min_delay);
    }

    public void DrotateArmTime(double timeout) {
        hardware.armMotorRotate.setPower(speed_half);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= timeout && !hardware.pressed(hardware.armLimitRotateDown)) {
            idle();
        }
        hardware.armMotorRotate.setPower(0);
        sleep(min_delay);
    }

    public void liftArmTime (double timeout) {
        hardware.armMotorLift.setPower(speed_slow);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= timeout && !hardware.pressed(hardware.armLimitLiftUp)) {
            idle();
        }
        hardware.armMotorLift.setPower(0);
        sleep(min_delay);
    }

    public void lowerArmTime (double timeout) {
        hardware.armMotorLift.setPower(-speed_slow);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= timeout && !hardware.pressed(hardware.armLimitLiftDown)) {
            idle();
        }
        hardware.armMotorLift.setPower(0);
        sleep(min_delay);
    }

    public void BackUP_stop(double speed, double timeout, double angle) {
        setDriveMotorMode_all(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.servo_frontLeft.setPosition(angle);
        hardware.servo_frontRight.setPosition(angle);
        hardware.servo_rearLeft.setPosition(angle);
        hardware.servo_rearRight.setPosition(angle);
        sleep(500);
        runtime.reset();
        setDriveMotorPower_all(speed);
        while (opModeIsActive() &&
                runtime.seconds() <= timeout &&
                hardware.backDistance.getDistance(DistanceUnit.CM) > 10) {
            idle();
        }
        setDriveMotorPower_all(0);
        setDriveMotorMode_all(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(min_delay);
    }


        public void vertical() {
            hardware.servo_frontLeft.setPosition(1);
            hardware.servo_frontRight.setPosition(0);
            hardware.servo_rearLeft.setPosition(1);
            hardware.servo_rearRight.setPosition(0);

            frontLeft = motorOrientation.reverse;
            frontRight = motorOrientation.forward;
            rearLeft = motorOrientation.reverse;
            rearRight = motorOrientation.forward;
            sleep(min_delay * 6);
        }

        public void horizontal() {
            hardware.servo_frontLeft.setPosition(0.5);
            hardware.servo_frontRight.setPosition(0.5);
            hardware.servo_rearLeft.setPosition(0.5);
            hardware.servo_rearRight.setPosition(0.5);

            frontLeft = motorOrientation.forward;
            frontRight = motorOrientation.forward;
            rearLeft = motorOrientation.forward;
            rearRight = motorOrientation.forward;
            sleep(min_delay * 6);
        }

        public void encoderDriveDistance(double speed, double distance, double timeout) {
            double encoderDistnace = distance * encoder_cm;

            encoderDrive(speed, encoderDistnace, encoderDistnace, encoderDistnace, encoderDistnace, timeout);
            sleep(min_delay);
        }

        public void encoderDrive(double power,
                                double frontLeftEncoderTicks,
                                double frontRightEncoderTicks,
                                double rearLeftEncoderTicks,
                                double rearRightEncoderTicks,
                                double timeout) {

            //Reset the encoders on the motors that we want to use
            hardware.motor_rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Make targets for encoder(s)
            int newFrontLeftTarget;
            int newFrontRightTarget;
            int newRearLeftTarget;
            int newRearRightTarget;

            if (opModeIsActive()) {
                idle();

                //Assign target values
                newFrontLeftTarget = hardware.motor_frontLeft.getCurrentPosition() + (int) frontLeftEncoderTicks;
                newFrontRightTarget = hardware.motor_frontRight.getCurrentPosition() + (int) frontRightEncoderTicks;
                newRearLeftTarget = hardware.motor_rearLeft.getCurrentPosition() + (int) rearLeftEncoderTicks;
                newRearRightTarget = hardware.motor_rearRight.getCurrentPosition() + (int) rearRightEncoderTicks;

                //Set the targets
                hardware.motor_frontLeft.setTargetPosition(newFrontLeftTarget);
                hardware.motor_frontRight.setTargetPosition(newFrontRightTarget);
                hardware.motor_rearLeft.setTargetPosition(newRearLeftTarget);
                hardware.motor_rearRight.setTargetPosition(newRearRightTarget);

                //Set the mode on encoders to run to position
                setDriveMotorMode_all(DcMotor.RunMode.RUN_TO_POSITION);

                //Reset the timeout and start motion
                runtime.reset();

                //power = Range.clip(Math.abs(power), 0.0, 1.0);
                double vpower = Range.clip(Math.abs(power), 0.0, 1.0);
                setDriveMotorPower_all(vpower);

                while (opModeIsActive() &&
                        (runtime.seconds() < timeout) &&
                        (hardware.motor_rearLeft.getCurrentPosition() <= newRearLeftTarget) &&
                        (hardware.motor_rearRight.getCurrentPosition() <= newRearRightTarget) &&
                        (hardware.motor_frontLeft.getCurrentPosition() <= newFrontLeftTarget) &&
                        (hardware.motor_frontRight.getCurrentPosition() <= newFrontRightTarget)) {
                    idle();
                }

                setDriveMotorPower_all(0);
                setDriveMotorMode_all(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

        }

        public void runToHit(DigitalChannel touchsensor, double speed, double timeout) {
            setDriveMotorMode_all(DcMotor.RunMode.RUN_USING_ENCODER);
            runtime.reset();
            setDriveMotorPower(-speed, speed, -speed, speed);
            hardware.motor_frontLeft.setPower(-speed);
            hardware.motor_frontRight.setPower(speed);
            hardware.motor_rearLeft.setPower(-speed);
            hardware.motor_rearRight.setPower(speed);
            while (opModeIsActive() && runtime.seconds() <= timeout && !hardware.pressed(touchsensor)) {
                telemetry.addData("touchy: ", "%b", touchsensor.getState());
                telemetry.update();
                idle();
            }
            setDriveMotorPower_all(0);
            setDriveMotorMode_all(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            sleep(min_delay);
        }

        public void runToUnHit(DigitalChannel touchsensor, double speed, double timeout) {
            setDriveMotorMode_all(DcMotor.RunMode.RUN_USING_ENCODER);
            runtime.reset();
            setDriveMotorPower(-speed, speed, -speed, speed);
            while (opModeIsActive() && runtime.seconds() <= timeout && hardware.pressed(touchsensor)) {
                telemetry.addData("touchy: ", "%b", touchsensor.getState());
                telemetry.update();
                idle();
            }
            setDriveMotorPower_all(0);
            setDriveMotorMode_all(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sleep(min_delay);
        }

        public void setDriveMotorMode(DcMotor.RunMode frontLeft, DcMotor.RunMode frontRight, DcMotor.RunMode rearLeft, DcMotor.RunMode rearRight) {
            hardware.motor_frontLeft.setMode(frontLeft);
            hardware.motor_frontRight.setMode(frontRight);
            hardware.motor_rearLeft.setMode(rearLeft);
            hardware.motor_rearRight.setMode(rearRight);
        }

        public void setDriveMotorMode_all(DcMotor.RunMode mode) {
            hardware.motor_frontLeft.setMode(mode);
            hardware.motor_frontRight.setMode(mode);
            hardware.motor_rearLeft.setMode(mode);
            hardware.motor_rearRight.setMode(mode);
        }

        public void setDriveMotorPower(double frontLeft, double frontRight, double rearLeft, double rearRight) {
            hardware.motor_frontLeft.setPower(frontLeft);
            hardware.motor_frontRight.setPower(frontRight);
            hardware.motor_rearLeft.setPower(rearLeft);
            hardware.motor_rearRight.setPower(rearRight);
        }

        public void setDriveMotorPower_all(double power) {
            hardware.motor_frontLeft.setPower(power);
            hardware.motor_frontRight.setPower(power);
            hardware.motor_rearLeft.setPower(power);
            hardware.motor_rearRight.setPower(power);
        }

}