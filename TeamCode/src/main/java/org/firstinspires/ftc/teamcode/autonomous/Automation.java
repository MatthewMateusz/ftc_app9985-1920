package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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

    //Delays
    static final long rotate_delay = 100;
    static final long drive_delay = 100;

    //Rotation stuff
    Orientation lastAngles = new Orientation();
    double globalAngle;

    //Drive straight stuff
    PIDController pidRotate = new PIDController(0.003, 0.0003, 0);
    PIDController pidDrive =  new PIDController(0.05, 0, 0);


    private ElapsedTime runtime = new ElapsedTime();

    Hardware hardware = new Hardware();


    @Override
    public final void runOpMode() throws InterruptedException {
        //Do universal init stuff
        hardware.init(hardwareMap);
        initIMU();

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

    private void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        hardware.imu.initialize(parameters);

        while (!hardware.imu.isGyroCalibrated() && !hardware.imu.isAccelerometerCalibrated()) {
            idle();
        }
    }

    private void resetAngle() {
        lastAngles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {
        Orientation angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        else {
            //Do nothing;
        }
        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }



    /*
    **
    */
    void rotate(double angle, double power, double timeout, boolean brake) {
        resetAngle();

        if (Math.abs(angle) > 359) angle = (int) Math.copySign(359, angle);

        pidRotate.reset();
        pidRotate.setSetpoint(angle);
        pidRotate.setInputRange(0, angle);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.setEnable(true);


        setBrakeMode(brake);
        if (opModeIsActive()) {
            runtime.reset();
            if (angle < 0) {
                while(opModeIsActive() && getAngle() == 0) {
                    hardware.motor_frontLeft.setPower(power);
                    hardware.motor_frontRight.setPower(power);
                    hardware.motor_rearLeft.setPower(-power);
                    hardware.motor_rearRight.setPower(-power);
                    idle();
                }

                do {
                    power = pidRotate.performPID(getAngle());
                    hardware.motor_frontLeft.setPower(power);
                    hardware.motor_frontRight.setPower(power);
                    hardware.motor_rearLeft.setPower(-power);
                    hardware.motor_rearRight.setPower(-power);
                    idle();
                } while (opModeIsActive() && !pidRotate.onTarget() && runtime.seconds() < timeout);
            } else {
                do {
                    power = pidRotate.performPID(getAngle());
                    hardware.motor_frontLeft.setPower(-power);
                    hardware.motor_frontRight.setPower(-power);
                    hardware.motor_rearLeft.setPower(power);
                    hardware.motor_rearRight.setPower(power);
                    idle();
                } while (opModeIsActive() && !pidRotate.onTarget() && runtime.seconds() < timeout);
            }

            telemetry.addData("test:", pidRotate.onTarget());
            telemetry.update();
            hardware.motor_frontLeft.setPower(0);
            hardware.motor_frontRight.setPower(0);
            hardware.motor_rearLeft.setPower(0);
            hardware.motor_rearRight.setPower(0);

            sleep(rotate_delay);
        }
    }


    /*
    **
    */
    void driveIMUDistance(double distance, double angle, double power, double timeout, boolean brake) {
        angle = Range.clip(angle, -180, 180);
        power = Range.clip(Math.abs(power), 0.0, 1.0);
        Position start = hardware.imu.getPosition();

        resetAngle();

        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.setEnable(true);

        setBrakeMode(brake);
        if (opModeIsActive()) {
            double frontLeft;
            double frontRight;
            double rearLeft;
            double rearRight;

            if (angle <= -90.0) {
                frontLeft  = -power;
                frontRight = ((-1/45) * angle - 3) * power;
                rearLeft   = ((-1/45) * angle - 3) * power;
                rearRight  = -power;
            } else if (-90.0 <= angle && angle <= 0) {
                frontLeft = ((1/45) * angle + 1) * power;
                frontRight = -power;
                rearLeft = -power;
                rearRight = ((1/45) * angle + 1) * power;
            } else if (0 <= angle && angle <= 90) {
                frontLeft = power;
                frontRight = ((1/45) * angle - 1) * power;
                rearLeft = ((1/45) * angle - 1) * power;
                rearRight = power;
            } else /*because of clip -> 90 <= angle <= 180*/{
                frontLeft = ((-1/45) * angle + 3) * power;
                frontRight = power;
                rearLeft = power;
                rearRight = ((-1/45) * angle + 3) * power;
            }

            runtime.reset();
            do {
                double correction = pidDrive.performPID(getAngle());
                hardware.motor_frontLeft.setPower(frontLeft + correction);
                hardware.motor_frontRight.setPower(frontRight + correction);
                hardware.motor_rearLeft.setPower(rearLeft - correction);
                hardware.motor_rearRight.setPower(rearRight - correction);
                idle();
            } while (opModeIsActive() && runtime.seconds() < timeout && distance(start, hardware.imu.getPosition()) <= distance);
        }

        hardware.motor_frontLeft.setPower(0);
        hardware.motor_frontRight.setPower(0);
        hardware.motor_rearLeft.setPower(0);
        hardware.motor_rearRight.setPower(0);

        resetAngle();
        sleep(drive_delay);
    }

    private double distance(Position start, Position end) {
        start.toUnit(DistanceUnit.CM);
        end.toUnit(DistanceUnit.CM);
        return Math.sqrt(((end.x - start.x) * (end.x - start.x))+((end.y - start.y) * (end.y - start.y))+((end.z - start.z) * (end.z - start.z)));
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

    void setBrakeMode(boolean brake) {
        if (brake) {
            hardware.motor_frontLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            hardware.motor_frontRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            hardware.motor_rearLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            hardware.motor_rearRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        } else {
            hardware.motor_frontLeft.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            hardware.motor_frontRight.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            hardware.motor_rearLeft.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
            hardware.motor_rearRight.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        }
    }

}
