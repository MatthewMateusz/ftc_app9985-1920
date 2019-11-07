package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AnalogTouchSensor {

    //What the the phone thinks the touch sensor is.
    private AnalogInput sensor;

    //The voltage at which the touch sensor is consider pressed.
    private static final double VOLTAGE_LIMIT = 0.5;


    //Initializes the AnalogInput
    public AnalogTouchSensor (HardwareMap hwMap, String phoneName) {
        this.sensor = hwMap.get(AnalogInput.class, phoneName);
    }


    //Checks if the button is pressed.
    //True = pressed
    //False = not pressed
    public boolean isPressed () {
        if (sensor.getVoltage() < VOLTAGE_LIMIT)
            return false;
        return true;
    }
}
