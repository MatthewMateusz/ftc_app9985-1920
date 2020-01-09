package org.firstinspires.ftc.teamcode.autonomous.modes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousFunction;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousFunctionGroup;
import org.firstinspires.ftc.teamcode.secure;

import java.util.ArrayList;
import java.util.List;

public abstract class Automation extends OpMode {
    public boolean useVuforia  = true;
    boolean terminate = false;

    Hardware hardware = new Hardware();
    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;

    enum skyStone {
        left,
        middle,
        right,
        unknown
    }

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String SKYSTONE = "Skystone";
    private static final String STONE = "Stone";

    public List<AutonomousFunction> activeFunction = new ArrayList<>();

    public void init() {
        setupHardware();
        if (useVuforia) {
            initVuforia();
            initTfod();
        }
        tfod.activate();
    }


    public void loop() {

    }

    public abstract void instruction();

    public void stop() {
        terminate = true;
        for (DcMotor motor : hardwareMap.dcMotor) {
            motor.setPower(0);
        }
    }

    //Instant functions
    void setupHardware() {
        hardware.init(hardwareMap);
    }

    void initVuforia () {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = secure.VUFORIA_KEY;
        parameters.cameraName = hardware.view;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    void initTfod() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(cameraMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, STONE, SKYSTONE);
    }
}
