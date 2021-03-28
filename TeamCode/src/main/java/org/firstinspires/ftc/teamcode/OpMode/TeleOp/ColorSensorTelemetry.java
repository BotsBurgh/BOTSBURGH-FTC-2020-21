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
        SmartColorSensor sensor;

        double redFudge, greenFudge, blueFudge;
        int red, green, blue;


        telemetry.addLine();
        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData(">", "Press stop");
            
            for (String key : Sensor.colorSensors.keySet()) {
                sensor = Sensor.getColorSensor(key);

                redFudge = sensor.getRedFudge();
                greenFudge = sensor.getGreenFudge();
                blueFudge = sensor.getBlueFudge();

                red   = (int) Range.clip(Sensor.getRed(Naming.COLOR_SENSOR_ARM) * 255 * redFudge, 0, 255);
                green = (int) Range.clip(Sensor.getGreen(Naming.COLOR_SENSOR_ARM) * 255 * greenFudge, 0, 255);
                blue  = (int) Range.clip(Sensor.getBlue(Naming.COLOR_SENSOR_ARM) * 255 * blueFudge, 0, 255);

                float[] hsv = new float[3];
                Color.RGBToHSV(red, green, blue, hsv);

                telemetry.addData(key + " Red", red);
                telemetry.addData(key + " Green", green);
                telemetry.addData(key + " Blue", blue);
                telemetry.addData(key + " Hue", hsv[0]);
                telemetry.addData(key + " Saturation", hsv[1]);
                telemetry.addData(key + " Value", hsv[2]);
                telemetry.addData(key + " Detected", Sensor.getRGB(key));
            }

            telemetry.update();
        }
    }
}
