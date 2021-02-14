package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
    public static double FUDGE = 10;
    public static double REDFUDGE   = 25*FUDGE;
    public static double GREENFUDGE = 15*FUDGE;
    public static double BLUEFUDGE  = 15*FUDGE;
    @Override
    public void runOpMode() throws InterruptedException {
        InitRobot.init(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine();
        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData(">", "Press stop");
            Sensor.Colors color = Robot.sensor.getRGB(Naming.COLOR_SENSOR_PARK, REDFUDGE, GREENFUDGE, BLUEFUDGE);
            if (color == Sensor.Colors.WHITE) {
                Robot.movement.move4x4(0,0,0,0);
                requestOpModeStop();
            } else {
                Robot.movement.move4x4(0.3,0.3,0.3,0.3);
            }
            telemetry.addData(">", color);
            telemetry.addData("Red", Robot.sensor.getRed(Naming.COLOR_SENSOR_PARK));
            telemetry.addData("Green", Robot.sensor.getRed(Naming.COLOR_SENSOR_PARK));
            telemetry.addData("Blue", Robot.sensor.getRed(Naming.COLOR_SENSOR_PARK));
            telemetry.update();

        }
    }
}