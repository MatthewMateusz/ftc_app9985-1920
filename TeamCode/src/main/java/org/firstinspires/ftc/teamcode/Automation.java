package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

abstract public class Automation extends LinearOpMode {

    //Variables ...


    enum direction {
        forward,
        left,
        right,
        backward
    }

    enum skyStone {
        left,
        middle,
        right,
        unkown
    }

    enum grabber {
        left,
        right,
        all
    }

    enum grabState {
        open,
        close
    }


    //NeveRest 40 Gearbox
     static final int encoder_tick_per_revolution = 280;

    //3-inch wheel (7.62cm)
    static final double adjust = 4;
    static final double encoder_cm = (encoder_tick_per_revolution / (7.62 * 3.141592653589793)) * adjust;

    //Tensor / Vuforia
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
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
    static final long min_delay = 50;

    //speed variables
    static final double speed_full = 1;
    static final double speed_normal = 0.6;
    static final double speed_half = 0.5;
    static final double speed_slow = 0.3;
    static final double speed_death = 0.1;

    //timeout
    static final double tShort = 3;
    static final double tMedium = 5;
    static final double tLong = 10;

    private ElapsedTime runtime = new ElapsedTime();
    Stopper stopper = new Stopper();

    Hardware hardware = new Hardware();

    //Setups the hardware for the hardware
    void setupHardware() {
        hardware.init(hardwareMap);
    }

    //Waits until the start button is pressed on the phone
    public void waitForStart() {
        while (!isStarted() && !isStopRequested()) {
            idle();
            telemetry.addData("Run Opmode", opModeIsActive());
            telemetry.update();
        }
        stopper.start();
    }

    void initVuforia () {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = secure.VUFORIA_KEY;
        parameters.cameraName = hardware.view;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        enabledVuforia = true;

    }

    void setGrabber(grabber which, grabState state) {
        if (which.equals(grabber.left)) {
            if (state.equals(grabState.close)) {
                hardware.servo_GrabberLeft.setPosition(1);
            } else {
                hardware.servo_GrabberLeft.setPosition(.2);
            }
        } else if (which.equals(grabber.right)) {
            if (state.equals(grabState.close)) {
                hardware.servo_GrabberRight.setPosition(1);
            } else {
                hardware.servo_GrabberRight.setPosition(.2);
            }
        } else {
            if (state.equals(grabState.close)) {
                hardware.servo_GrabberLeft.setPosition(1);
                hardware.servo_GrabberRight.setPosition(1);
            } else {
                hardware.servo_GrabberRight.setPosition(.2);
                hardware.servo_GrabberLeft.setPosition(.2);
            }
        }
        sleep(200);
    }

    void liftArm(boolean up, double timeout){
        runtime.reset();
        if (up) {
            hardware.armMotorLift.setPower(.75);
            while (stopper.canRun() && opModeIsActive() && !hardware.pressed(hardware.armLimtLiftUp) && runtime.seconds() < timeout) {
                idle();
            }
            hardware.armMotorLift.setPower(0);
        } else {
            hardware.armMotorLift.setPower(-.4);
            while(stopper.canRun() && opModeIsActive() && !hardware.pressed(hardware.armLimitLiftDown) && runtime.seconds() < timeout) {
                idle();
            }
            hardware.armMotorLift.setPower(0);
        }
    }

    void RotateArm(boolean counterClockwise, double timeout) {
        runtime.reset();
        if (counterClockwise) {
            while (stopper.canRun() && !hardware.pressed(hardware.armLimitRotateDown) && opModeIsActive() && timeout > runtime.seconds()) {
                hardware.armMotorRotate.setPower(.6);
            }
        } else {
            while (stopper.canRun() && !hardware.pressed(hardware.armLimitRotateUp) && opModeIsActive() && timeout > runtime.seconds()) {
                hardware.armMotorRotate.setPower(-.6);
            }
        }
        hardware.armMotorRotate.setPower(0);
        sleep(50);
    }

    void distanceDrive(direction movement, double distance, double power, double timeout, boolean brake) {
        double encoderDistance = distance * encoder_cm;
        double frontLeft;
        double frontRight;
        double rearLeft;
        double rearRight;
        if (movement.equals(direction.forward)) {
            frontLeft  = -encoderDistance;
            frontRight = encoderDistance;
            rearLeft   = -encoderDistance;
            rearRight  = encoderDistance;
        } else if (movement.equals(direction.left)) {
            frontLeft  = encoderDistance;
            frontRight = -encoderDistance;
            rearLeft   = encoderDistance;
            rearRight  = -encoderDistance;
        } else if (movement.equals(direction.right)) {
            frontLeft  = encoderDistance;
            frontRight = encoderDistance;
            rearLeft   = -encoderDistance;
            rearRight  = -encoderDistance;
        } else {
            frontLeft  = -encoderDistance;
            frontRight = -encoderDistance;
            rearLeft   = encoderDistance;
            rearRight  = encoderDistance;
        }
        if (brake) {
            hardware.motor_frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.motor_frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.motor_rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.motor_rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            hardware.motor_frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            hardware.motor_frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            hardware.motor_rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            hardware.motor_rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        encoderDrive(frontLeft, frontRight, rearLeft, rearRight, power, timeout);
    }

    private void encoderDrive(
            double frontLeftTicks,
            double frontRightTicks,
            double rearLeftTicks,
            double rearRightTicks,
            double power,
            double timeout) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        if (opModeIsActive()) {
            //Get new targets
            newFrontLeftTarget = hardware.motor_frontLeft.getCurrentPosition() + (int) frontLeftTicks;
            newFrontRightTarget = hardware.motor_frontRight.getCurrentPosition() + (int) frontRightTicks;
            newRearLeftTarget = hardware.motor_rearLeft.getCurrentPosition() + (int) rearLeftTicks;
            newRearRightTarget = hardware.motor_rearRight.getCurrentPosition() + (int) rearRightTicks;

            //Set new targets
            hardware.motor_frontLeft.setTargetPosition(newFrontLeftTarget);
            hardware.motor_frontRight.setTargetPosition(newFrontRightTarget);
            hardware.motor_rearLeft.setTargetPosition(newRearLeftTarget);
            hardware.motor_rearRight.setTargetPosition(newRearRightTarget);

            setDriveMotorMode_all(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();

            power = Range.clip(Math.abs(power), 0.0, 1.0);
            setDriveMotorPower_all(power);

            while (stopper.canRun() && opModeIsActive() &&
            runtime.seconds() < timeout &&
            notAtTarget(hardware.motor_frontLeft.getCurrentPosition(), newFrontLeftTarget) &&
            notAtTarget(hardware.motor_frontRight.getCurrentPosition(), newFrontRightTarget) &&
            notAtTarget(hardware.motor_rearLeft.getCurrentPosition(), newRearLeftTarget) &&
            notAtTarget(hardware.motor_rearRight.getCurrentPosition(), newRearRightTarget)) {
                //Do nothing
            }
            setDriveMotorPower_all(0);
        }
    }


    void initTfod() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(cameraMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, STONE, SKYSTONE);
        enabledTfod = true;
    }

    skyStone testBlockThing(long delay, double scanningTime) {
        tfod.activate();
        int left = 0;
        int middle = 0;
        int right = 0;
        runtime.reset();
        while( stopper.canRun() && opModeIsActive() && runtime.seconds() <= scanningTime) {
            List<Recognition> updatedRecongnitions = tfod.getUpdatedRecognitions();
            if (updatedRecongnitions != null) {
                for (Recognition recon : updatedRecongnitions) {
                    telemetry.addData("Object: ", recon.getLabel());
                }
                if (updatedRecongnitions.size() == 3) {
                    float skystonePosition = 0;
                    boolean skystone = false;
                    float stone1Position = 0;
                    boolean stone1 = false;
                    float stone2Position = 0;
                    boolean stone2 = false;

                    int stoneNumber = 0;
                    for (Recognition recon : updatedRecongnitions) {
                        if (recon.getLabel().equals(SKYSTONE)) {
                            skystonePosition =recon.getLeft();
                            skystone = true;
                        } else {
                            if (stoneNumber == 0) {
                                stone1Position = recon.getLeft();
                                stoneNumber++;
                                stone1 = true;
                            } else {
                                stone2Position = recon.getLeft();
                                stoneNumber++;
                                stone2 = true;
                            }
                        }
                    }
                    if (skystone && stone1 && stone2) {
                        //Ensure the smaller stone position value is in 1
                        if (stone2Position < stone1Position) {
                            float temp = stone2Position;
                            stone2Position = stone1Position;
                            stone1Position = temp;
                        }

                        if ((skystonePosition < stone1Position) && (skystonePosition < stone2Position)) {
                            telemetry.addData("Skystone location:", "left");
                            left++;
                        } else if ((skystonePosition > stone1Position) && (skystonePosition < stone2Position)) {
                            telemetry.addData("Skystone location:", "middle");
                            middle++;
                        } else if (skystonePosition > stone1Position && skystonePosition > stone2Position) {
                            telemetry.addData("Skystone locatoin:", "right");
                            right++;
                        } else {
                            telemetry.addData("Skystone location", "I'm failing you");
                        }
                    } else {
                        telemetry.addData("Skystone location:", "Undeterminable");
                    }

                }
                telemetry.update();
            }
            sleep(delay);
        }
        tfod.shutdown();
        if (left > middle) {
            if (left > right) {
                return skyStone.left;
            } else {
                return skyStone.right;
            }
        } else {
            if (middle > right) {
                return skyStone.middle;
            } else {
                return skyStone.right;
            }
        }
    }

    private Boolean notAtTarget(int current, int destination) {
        if (destination > 0) {
            return current <= destination;
        } else {
            return current >= destination;
        }
    }

    private void setDriveMotorMode(DcMotor.RunMode frontLeft, DcMotor.RunMode frontRight, DcMotor.RunMode rearLeft, DcMotor.RunMode rearRight) {
        hardware.motor_frontLeft.setMode(frontLeft);
        hardware.motor_frontRight.setMode(frontRight);
        hardware.motor_rearLeft.setMode(rearLeft);
        hardware.motor_rearRight.setMode(rearRight);
    }

    private void setDriveMotorMode_all(DcMotor.RunMode mode) {
        hardware.motor_frontLeft.setMode(mode);
        hardware.motor_frontRight.setMode(mode);
        hardware.motor_rearLeft.setMode(mode);
        hardware.motor_rearRight.setMode(mode);
    }

    private void setDriveMotorPower(double frontLeft, double frontRight, double rearLeft, double rearRight) {
        hardware.motor_frontLeft.setPower(frontLeft);
        hardware.motor_frontRight.setPower(frontRight);
        hardware.motor_rearLeft.setPower(rearLeft);
        hardware.motor_rearRight.setPower(rearRight);
    }

    private void setDriveMotorPower_all(double power) {
        hardware.motor_frontLeft.setPower(power);
        hardware.motor_frontRight.setPower(power);
        hardware.motor_rearLeft.setPower(power);
        hardware.motor_rearRight.setPower(power);
    }

    void STOP() {
        setDriveMotorPower_all(0);
        hardware.armMotorLift.setPower(0);
        hardware.armMotorRotate.setPower(0);
    }

}