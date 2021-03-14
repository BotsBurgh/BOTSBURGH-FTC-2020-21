package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.HW.SmartColorSensor;
import org.firstinspires.ftc.teamcode.API.InitRobot;
import org.firstinspires.ftc.teamcode.API.Robot;
import org.firstinspires.ftc.teamcode.API.Sensor;

/*
 * This is a simple program to reach the white line
 */
@Config
@TeleOp(name="Color Sensor Telemetry", group="99-test")
public class ColorSensorTelemetry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        InitRobot.init(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SmartColorSensor sensor = Sensor.getColorSensor(Naming.COLOR_SENSOR_PARK);

        double redFudge = sensor.getRedFudge();
        double greenFudge = sensor.getGreenFudge();
        double blueFudge = sensor.getBlueFudge();


        telemetry.addLine();
        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData(">", "Press stop");
            int red   = (int) Range.clip(Sensor.getRed(Naming.COLOR_SENSOR_PARK) * 255 * redFudge, 0, 255);
            int green = (int) Range.clip(Sensor.getGreen(Naming.COLOR_SENSOR_PARK) * 255 * greenFudge, 0, 255);
            int blue  = (int) Range.clip(Sensor.getBlue(Naming.COLOR_SENSOR_PARK) * 255 * blueFudge, 0, 255);

            float[] hsv = new float[3];
            Color.RGBToHSV(red, green, blue, hsv);

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Hue", hsv[0]);
            telemetry.addData("Saturation", hsv[1]);
            telemetry.addData("Value", hsv[2]);
            telemetry.addData("Detected", Sensor.getRGB(Naming.COLOR_SENSOR_PARK));

            telemetry.update();
        }
    }
}
