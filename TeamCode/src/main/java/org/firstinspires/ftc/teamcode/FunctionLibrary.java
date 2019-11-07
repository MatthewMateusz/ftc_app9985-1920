package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

abstract public class FunctionLibrary  extends LinearOpMode {

    //Variables ...

    enum motorOrientation {
        forward,
        reverse
    }

    motorOrientation frontLeft = motorOrientation.forward;
    motorOrientation frontRight = motorOrientation.forward;
    motorOrientation rearLeft = motorOrientation.forward;
    motorOrientation rearRight = motorOrientation.forward;


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
    servo servo = new servo();
    motor motor = new motor();

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

    public void rotateArmTime(double timeout) {
        hardware.armMotorRotate.setPower(-speed_half);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= timeout && !hardware.pressed(hardware.armLimitRotateUp)) {
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

    public void BackUP_stop(double speed, double timeout) {
        hardware.motor_rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.motor_rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.motor_frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.motor_frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();
        hardware.motor_frontLeft.setPower(speed);
        hardware.motor_frontRight.setPower(-speed);
        hardware.motor_rearLeft.setPower(speed);
        hardware.motor_rearRight.setPower(-speed);
        while (opModeIsActive() && runtime.seconds() <= timeout && hardware.backDistance.getDistance(DistanceUnit.CM) > 10) {
            idle();
        }
        hardware.motor_frontLeft.setPower(0);
        hardware.motor_frontRight.setPower(0);
        hardware.motor_rearLeft.setPower(0);
        hardware.motor_rearRight.setPower(0);
        hardware.motor_frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.motor_frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.motor_rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.motor_rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(min_delay);
    }


    class servo {
        public void vertical() {
            hardware.servo_frontLeft.setPosition(0);
            hardware.servo_frontRight.setPosition(1);
            hardware.servo_rearLeft.setPosition(0);
            hardware.servo_rearRight.setPosition(1);

            frontLeft = motorOrientation.forward;
            frontRight = motorOrientation.reverse;
            rearLeft = motorOrientation.forward;
            rearRight = motorOrientation.reverse;
            sleep(min_delay * 5);
        }

        public void horizontal() {
            hardware.servo_frontLeft.setPosition(0.5);
            hardware.servo_frontRight.setPosition(0.5);
            hardware.servo_rearLeft.setPosition(0.5);
            hardware.servo_rearRight.setPosition(0.5);

            frontLeft = motorOrientation.reverse;
            frontRight = motorOrientation.forward;
            rearLeft = motorOrientation.reverse;
            rearRight = motorOrientation.forward;
            sleep(min_delay * 5);
        }
    }

    class motor {
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
                newFrontLeftTarget = hardware.motor_rearLeft.getCurrentPosition() + (int) rearLeftEncoderTicks * 100;
                newFrontRightTarget = hardware.motor_frontRight.getCurrentPosition() + (int) frontRightEncoderTicks * 100;
                newRearLeftTarget = hardware.motor_rearLeft.getCurrentPosition() + (int) rearLeftEncoderTicks;
                newRearRightTarget = hardware.motor_frontLeft.getCurrentPosition() + (int) frontLeftEncoderTicks * 100;

                //Set the targets


                hardware.motor_rearLeft.setTargetPosition(newRearLeftTarget);
                hardware.motor_frontLeft.setTargetPosition(newFrontLeftTarget);
                if (rearLeft.equals(motorOrientation.reverse)) {
                    hardware.motor_rearRight.setTargetPosition(newRearRightTarget);
                    hardware.motor_frontRight.setTargetPosition(newFrontRightTarget);
                } else {
                    hardware.motor_rearRight.setTargetPosition(-newRearRightTarget);
                    hardware.motor_frontRight.setTargetPosition(-newFrontRightTarget);
                }



                //Set the mode on encoders to run to position
                hardware.motor_rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.motor_rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.motor_frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.motor_frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Reset the timeout and start motion
                runtime.reset();


                //power = Range.clip(Math.abs(power), 0.0, 1.0);
                double vpower = Range.clip(Math.abs(power), 0.0, 1.0);
                hardware.motor_rearLeft.setPower(vpower);
                hardware.motor_rearRight.setPower(vpower);
                hardware.motor_frontLeft.setPower(vpower);
                hardware.motor_frontRight.setPower(vpower);


                idle();

                while (opModeIsActive() && (runtime.seconds() < timeout) && (hardware.motor_rearLeft.isBusy())) {
                    idle();
                }

                hardware.motor_frontLeft.setPower(0);
                hardware.motor_frontRight.setPower(0);
                hardware.motor_rearLeft.setPower(0);
                hardware.motor_rearRight.setPower(0);

                hardware.motor_frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hardware.motor_frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hardware.motor_rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hardware.motor_rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

        }
        public void liftArm(double timeout, double speed) {
            runtime.reset();
            double power = Math.abs(speed);
            //set motor to power

            while (opModeIsActive() && runtime.seconds() <= timeout) {
                idle();
            }

            //set motor to 0
        }

        public void runToHit(DigitalChannel touchsensor, double speed, double timeout) {
            hardware.motor_rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.motor_rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.motor_frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.motor_frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            runtime.reset();
            hardware.motor_frontLeft.setPower(speed);
            hardware.motor_frontRight.setPower(-speed);
            hardware.motor_rearLeft.setPower(speed);
            hardware.motor_rearRight.setPower(-speed);
            while (opModeIsActive() && runtime.seconds() <= timeout && !hardware.pressed(touchsensor)) {
                telemetry.addData("touchy: ", "%b", touchsensor.getState());
                telemetry.update();
                idle();
            }
            hardware.motor_frontLeft.setPower(0);
            hardware.motor_frontRight.setPower(0);
            hardware.motor_rearLeft.setPower(0);
            hardware.motor_rearRight.setPower(0);
            hardware.motor_frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.motor_frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.motor_rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.motor_rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sleep(min_delay);
        }

        public void runToUnHit(DigitalChannel touchsensor, double speed, double timeout) {
            hardware.motor_rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.motor_rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.motor_frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.motor_frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            runtime.reset();
            hardware.motor_frontLeft.setPower(speed);
            hardware.motor_frontRight.setPower(-speed);
            hardware.motor_rearLeft.setPower(speed);
            hardware.motor_rearRight.setPower(-speed);
            while (opModeIsActive() && runtime.seconds() <= timeout && hardware.pressed(touchsensor)) {
                telemetry.addData("touchy: ", "%b", touchsensor.getState());
                telemetry.update();
                idle();
            }
            hardware.motor_frontLeft.setPower(0);
            hardware.motor_frontRight.setPower(0);
            hardware.motor_rearLeft.setPower(0);
            hardware.motor_rearRight.setPower(0);
            hardware.motor_frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.motor_frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.motor_rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.motor_rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sleep(min_delay);
        }
    }
}