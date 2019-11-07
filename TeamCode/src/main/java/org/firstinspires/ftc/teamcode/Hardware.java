package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Hardware {

    //Variables to improve readability of code
    public DcMotor.Direction forwardDirection = DcMotor.Direction.FORWARD;
    public DcMotor.Direction reverseDirection = DcMotor.Direction.REVERSE;

    public DcMotor.ZeroPowerBehavior brakeZero = DcMotor.ZeroPowerBehavior.BRAKE;
    public DcMotor.ZeroPowerBehavior floatZero = DcMotor.ZeroPowerBehavior.FLOAT;


    //Default init variables. What servos and other hardware init to
    public static final double initPosition = 0.5;

    //Hardware

    //Motor
    public DcMotor motor_frontLeft = null;
    public DcMotor motor_frontRight = null;
    public DcMotor motor_rearLeft = null;
    public DcMotor motor_rearRight = null;
    public DcMotor armMotorRotate = null;
    public DcMotor armMotorLift = null;



    //Servo
    public Servo servo_frontLeft = null;
    public Servo servo_frontRight = null;
    public Servo servo_rearLeft = null;
    public Servo servo_rearRight = null;
    public CRServo armServoBottom = null;
    public CRServo armServoTop = null;

    //Sensors

    public DigitalChannel armLimitRotateUp = null;
    public DigitalChannel armLimitRotateDown = null;
    public DigitalChannel armLimitLiftUp = null;
    public DigitalChannel armLimitLiftDown = null;

    public Rev2mDistanceSensor backDistance= null;

    HardwareMap hwMap = null;

    public Hardware() {

    }

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        motor_frontLeft = setupMotor(hwMap,"motor_frontLeft", forwardDirection, brakeZero);
        motor_frontRight = setupMotor(hwMap,"motor_frontRight", forwardDirection, brakeZero);
        motor_rearLeft = setupMotor(hwMap,"motor_rearLeft", forwardDirection, brakeZero);
        motor_rearRight = setupMotor(hwMap,"motor_rearRight", forwardDirection, brakeZero);
        armMotorRotate = setupMotor(hwMap,"armMotorRotate", forwardDirection, brakeZero);
        armMotorLift = setupMotor(hwMap,"armMotorLift",forwardDirection,brakeZero);

        servo_frontLeft = setupServo(hwMap, "servo_frontLeft", initPosition);
        servo_frontRight = setupServo(hwMap, "servo_frontRight", initPosition);
        servo_rearLeft = setupServo(hwMap, "servo_rearLeft", initPosition);
        servo_rearRight = setupServo(hwMap, "servo_rearRight", initPosition);
        armServoBottom = setupContinuousServo(hwMap,"armServoTop",0.03);
        armServoTop = setupContinuousServo(hwMap,"armServoTop",0.03);

        armLimitRotateUp = hwMap.get(DigitalChannel.class, "armLimitRotateUp");
        armLimitRotateDown = hwMap.get(DigitalChannel.class,"armLimitRotateDown");
        armLimitLiftUp = hwMap.get(DigitalChannel.class,"armLimitLiftUp");
        armLimitLiftDown = hwMap.get(DigitalChannel.class, "armLimitLiftDown");

        backDistance = (Rev2mDistanceSensor) hwMap.get(DistanceSensor.class, "backDistanceSensor");
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
