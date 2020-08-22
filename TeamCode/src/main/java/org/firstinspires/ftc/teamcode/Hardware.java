package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Hardware {

    //Motors
    public DcMotor frontLeft  = null;
    public DcMotor frontRight = null;
    public DcMotor rearLeft   = null;
    public DcMotor rearRight  = null;

    //Servo
    public Servo test_servo = null;

    //Sensors



    static HardwareMap hwMap = null;


    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        frontLeft = setupMotor(
                hwMap,
                "",
                DcMotor.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        frontRight = setupMotor(hwMap,
                "",
                DcMotor.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rearLeft = setupMotor(
                hwMap,
                "",
                DcMotor.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rearRight = setupMotor(
                hwMap,
                "",
                DcMotor.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        test_servo = setupServo(
                hwMap,
                "",
                Servo.Direction.FORWARD,
                0,
                0,
                1
        );
    }




    private DcMotor setupMotor(HardwareMap hwMap, String phoneName, DcMotor.Direction motorDirection, DcMotor.ZeroPowerBehavior zeroPower) {
        DcMotor motor = hwMap.get(DcMotor.class, phoneName);
        motor.setDirection(motorDirection);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(zeroPower);
        return motor;
    }

    private static Servo setupServo(HardwareMap hwMap, String phoneName, Servo.Direction direction, double position, double min, double max) {
        Servo servo = hwMap.get(Servo.class, phoneName);
        servo.setPosition(position);
        servo.setDirection(direction);
        servo.scaleRange(min, max);
        return servo;
    }

    private static CRServo setupContinuousServo(HardwareMap hwMap, String phoneName, CRServo.Direction direction, double speed) {
        CRServo servo = hwMap.crservo.get(phoneName);
        servo.setPower(speed);
        servo.setDirection(direction);
        return servo;
    }
}
