package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.API.InitRobot;
import org.firstinspires.ftc.teamcode.API.Robot;
import org.firstinspires.ftc.teamcode.API.Sensor;

/*
 * This is a simple program to reach the white line
 */
@Config
@Autonomous(group = "drive")
public class WhiteLine extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        InitRobot.init(this);

        telemetry.addLine();
        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData(">", "Press stop");
            telemetry.update();
            if (Robot.sensor.getRGB("line", 1000, 1000, 1000) == Sensor.Colors.WHITE) {
                Robot.movement.move4x4(0.6,0.6,0.6,0.6);
            } else {
                Robot.movement.move4x4(0,0,0,0);
                requestOpModeStop();
            }
        }
    }
}