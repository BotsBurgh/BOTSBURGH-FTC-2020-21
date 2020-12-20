package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.InitRobot;
import org.firstinspires.ftc.teamcode.API.Robot;

/*
 * This is a simple program to reach the white line
 */
@Config
@TeleOp(name="Color Sensor Telemetry")
public class ColorSensorTelemetry extends LinearOpMode {
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
            int red   = (int) Range.clip(Robot.sensor.getRed(Naming.COLOR_SENSOR_PARK) * 255 * REDFUDGE, 0, 255);
            int green = (int) Range.clip(Robot.sensor.getGreen(Naming.COLOR_SENSOR_PARK) * 255 * GREENFUDGE, 0, 255);
            int blue  = (int) Range.clip(Robot.sensor.getBlue(Naming.COLOR_SENSOR_PARK) * 255 * BLUEFUDGE, 0, 255);

            float[] hsv = new float[3];
            Color.RGBToHSV(red, green, blue, hsv);

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Hue", hsv[0]);
            telemetry.addData("Saturation", hsv[1]);
            telemetry.addData("Value", hsv[2]);
            telemetry.addData("Detected", Robot.sensor.getRGB(Naming.COLOR_SENSOR_PARK, REDFUDGE, GREENFUDGE, BLUEFUDGE));

            telemetry.update();
        }
    }
}
