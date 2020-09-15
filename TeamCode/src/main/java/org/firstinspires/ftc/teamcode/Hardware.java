package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Hardware {

    //Default init variables. What servos and other hardware init to
    public static final double initPosition = 0.5;

    //Hardware

    //Motor
    public DcMotor motor_frontLeft = null;
    public DcMotor motor_frontRight = null;
    public DcMotor motor_rearLeft = null;
    public DcMotor motor_rearRight = null;




    //Servo

    public CRServo servo_left_cont = null;

    //Sensors
    public BNO055IMU imu;

    public DigitalChannel touch_Front = null;
    public DigitalChannel touch_Rear = null;

    WebcamName view;

    HardwareMap hwMap = null;

    public Hardware() {

    }

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        motor_frontLeft = setupMotor(hwMap,"motor_frontLeft", Direction.REVERSE, ZeroPowerBehavior.BRAKE);
        motor_frontRight = setupMotor(hwMap,"motor_frontRight", Direction.FORWARD, ZeroPowerBehavior.BRAKE);
        motor_rearLeft = setupMotor(hwMap,"motor_rearLeft", Direction.FORWARD, ZeroPowerBehavior.BRAKE);
        motor_rearRight = setupMotor(hwMap,"motor_rearRight", Direction.REVERSE, ZeroPowerBehavior.BRAKE);



        servo_left_cont = setupContinuousServo(hwMap,"servo_left_cont",-.05);

        touch_Front = hwMap.get(DigitalChannel.class, "touch_front");
        touch_Rear = hwMap.get(DigitalChannel.class,"touch_rear");

        imu = hwMap.get(BNO055IMU.class, "imu");



    }

    private DcMotor setupMotor(HardwareMap hwMap, String phoneName, DcMotor.Direction motorDirection, DcMotor.ZeroPowerBehavior zeroPower) {
        DcMotor motor = hwMap.get(DcMotor.class, phoneName);
        motor.setDirection(motorDirection);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(zeroPower);
        return motor;
    }

    private Servo setupServo(HardwareMap hwMap, String phoneName, double position) {
        Servo servo = hwMap.get(Servo.class, phoneName);
        servo.setPosition(position);
        return servo;

    }

    private CRServo setupContinuousServo(HardwareMap hwMap, String phoneName, double speed){
        CRServo servo = hwMap.crservo.get(phoneName);
        servo.setPower(speed);
        return servo;
    }

    public boolean pressed(DigitalChannel touchsensor) {
        return !touchsensor.getState();
    }
}