package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.InitRobot;
import org.firstinspires.ftc.teamcode.API.Sensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class TFODTest0 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        InitRobot.init(this);
        Sensor.initVuforia(Naming.WEBCAM_0);
        Sensor.initTfod();

        waitForStart();

        telemetry.addData("Detected:", Sensor.detectDisks());
        telemetry.update();
    }
}