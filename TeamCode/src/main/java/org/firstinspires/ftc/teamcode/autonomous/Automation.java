package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

public abstract class Automation extends LinearOpMode {

    //Vars

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

    private ElapsedTime runtime = new ElapsedTime();

    Hardware hardware = new Hardware();


    @Override
    public final void runOpMode() throws InterruptedException {
        //Do universal init stuff
        hardware.init(hardwareMap);

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
    public abstract void instruction();

    /*
    ** This method is where the user puts its custom init code.
    ** It also holds init code that is not required for each autonomous for example:
    ** Vuforia or neural networks
    */
    public void auto_init() {};


}
