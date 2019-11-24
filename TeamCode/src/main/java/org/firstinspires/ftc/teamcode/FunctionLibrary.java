package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

abstract public class FunctionLibrary  extends LinearOpMode {

    //Variables ...





    enum motorOrientation {
        forward,
        reverse
    }

    enum capStone {
        left,
        middle,
        right,
        unkown
    }

    private motorOrientation frontLeft;
    private motorOrientation frontRight;
    private motorOrientation rearLeft;
    private motorOrientation rearRight;


    //NeveRest 40 Gearbox
     static final int encoder_tick_per_revolution = 280;

    //3-inch wheel (7.62cm)
    static final double adjust = 4;
    static final double encoder_cm = (encoder_tick_per_revolution / (7.62 * 3.141592653589793)) * adjust;

    //Tensor / Vuforia
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String SKYSTONE = "Skystone";
    private static final String STONE = "Stone";
    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;

    private Boolean enabledVuforia = false;
    private Boolean enabledTfod = false;

    //distance variables
    static final double inch_to_cm = 2.54;
    static final double one_tile = 24 * inch_to_cm;
    static final long min_delay = 100;

    //speed variables
    static final double speed_full = 1;
    static final double speed_normal = 0.7;
    static final double speed_half = 0.5;
    static final double speed_slow = 0.3;
    static final double speed_death = 0.1;

    //timeout
    static final double tShort = 3;
    static final double tMedium = 5;
    static final double tLong = 10;

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

    void initVuforia () {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = secure.VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        enabledVuforia = true;

    }

    void initTfod() {
        initVuforia();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(cameraMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, STONE, SKYSTONE);
        enabledTfod = true;
    }

    void testBlockThing(long delay) {
        tfod.activate();
        while(opModeIsActive()) {
            List<Recognition> updatedRecongnitions = tfod.getUpdatedRecognitions();
            if (updatedRecongnitions != null) {
                for (Recognition recon : updatedRecongnitions) {
                    telemetry.addData("Object: ", recon.getLabel());
                }
                telemetry.update();
            }
            sleep(delay);
        }
        tfod.shutdown();
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
            setDriveMotorMode_all(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Make targets for encoder(s)
            int newFrontLeftTarget;
            int newFrontRightTarget;
            int newRearLeftTarget;
            int newRearRightTarget;


            if (opModeIsActive()) {
                idle();

                //Assign target values
                if (frontLeft.equals(motorOrientation.forward)) {
                    newFrontLeftTarget = hardware.motor_frontLeft.getCurrentPosition() + (int) frontLeftEncoderTicks;
                } else {
                    newFrontLeftTarget = hardware.motor_frontLeft.getCurrentPosition() + (int) frontLeftEncoderTicks * -1;
                }

                if (frontRight.equals(motorOrientation.forward)) {
                    newFrontRightTarget = hardware.motor_frontRight.getCurrentPosition() + (int) frontRightEncoderTicks;
                } else {
                    newFrontRightTarget = hardware.motor_frontRight.getCurrentPosition() + (int) frontRightEncoderTicks * -1;
                }

                if (rearLeft.equals(motorOrientation.forward)) {
                    newRearLeftTarget = hardware.motor_rearLeft.getCurrentPosition() + (int) rearLeftEncoderTicks;
                } else {
                    newRearLeftTarget = hardware.motor_rearLeft.getCurrentPosition() + (int) rearLeftEncoderTicks * -1;
                }

                if (rearRight.equals(motorOrientation.forward)) {
                    newRearRightTarget = hardware.motor_rearRight.getCurrentPosition() + (int) rearRightEncoderTicks;
                } else {
                    newRearRightTarget = hardware.motor_rearRight.getCurrentPosition() + (int) rearRightEncoderTicks * -1;
                }


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
                        notAtTarget(hardware.motor_rearLeft.getCurrentPosition(), newFrontLeftTarget ) &&
                        notAtTarget(hardware.motor_rearRight.getCurrentPosition(), newRearRightTarget) &&
                        notAtTarget(hardware.motor_frontLeft.getCurrentPosition(), newFrontLeftTarget) &&
                        notAtTarget(hardware.motor_frontRight.getCurrentPosition(), newFrontRightTarget)) {
                    idle();
                }

                setDriveMotorPower_all(0);
                setDriveMotorMode_all(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

        }

        private Boolean notAtTarget(int current, int destination) {
            if (destination > 0) {
                if (current <= destination) {
                    return true;
                } else {
                    return false;
                }
            } else {
                if (current >= destination) {
                    return true;
                } else {
                    return false;
                }
            }
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