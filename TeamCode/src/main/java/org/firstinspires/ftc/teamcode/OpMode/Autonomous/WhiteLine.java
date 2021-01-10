package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.InitRobot;
import org.firstinspires.ftc.teamcode.API.Robot;
import org.firstinspires.ftc.teamcode.API.Sensor;

/*
 * This is a simple program to reach the white line
 */
@Config
@Autonomous(name="White Line", group = "drive")
public class WhiteLine extends LinearOpMode {
    private static final double REDFUDGE   = 25*30;
    private static final double GREENFUDGE = 15*30;
    private static final double BLUEFUDGE  = 15*30;
    @Override
    public void runOpMode() throws InterruptedException {
        InitRobot.init(this);

        telemetry.addLine();
        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData(">", "Press stop");
            telemetry.update();
            if (Robot.sensor.getRGB(Naming.COLOR_SENSOR_PARK, REDFUDGE, GREENFUDGE, BLUEFUDGE) == Sensor.Colors.WHITE) {
                Robot.movement.move4x4(0,0,0,0);
                requestOpModeStop();
            } else {
                Robot.movement.move4x4(0.3,0.3,0.3,0.3);
            }
        }
    }
}