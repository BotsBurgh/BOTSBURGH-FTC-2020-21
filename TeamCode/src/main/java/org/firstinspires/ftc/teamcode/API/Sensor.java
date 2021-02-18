/*
 * Copyright 2020 FIRST Tech Challenge Team 11792
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.API;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.API.HW.Encoder;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

import lombok.Getter;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * The Sensor class.
 * This is a class which interfaces with sensors, so you don't have to. See the README for more
 * information.
 */
public class Sensor {
    // Potentiometer configuration
    private static final int    POT_MAX = 270;   // Max range in degrees
    private static final double Vmax    = 0.004; // Minimum voltage
    private static final double Vmin    = 3.304; // Maximum voltage

    // VuForia configuration
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK; // Back camera or front
    private static final boolean PHONE_IS_PORTRAIT = true; // Set to true because our camera is rotated at 90 degrees
    private final float CAMERA_FORWARD_DISPLACEMENT  = 7.88f  * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
    private final float CAMERA_VERTICAL_DISPLACEMENT = 4.5f   * mmPerInch;   // eg: Camera is 8 Inches above ground
    private final float CAMERA_LEFT_DISPLACEMENT     = 5.625f * mmPerInch;   // eg: Camera is ON the robot's center line

    // Phone configuration
    private static final float phoneXRotate    = 0;
    private static final float phoneZRotate    = 9.5f;

    // Color sensor configuration
    // TODO: implement
    private static final double BLACK_THRESH  = 0.2;
    private static final double WHITE_THRESH  = 0.9;
    private static final double RED_THRESH    = 600;
    private static final double ORANGE_THRESH = 600;
    private static final double YELLOW_THRESH = 600;
    private static final double GREEN_THRESH  = 600;
    private static final double BLUE_THRESH   = 600;
    private static final double PURPLE_THRESH = 600;

    private static final double THRESH_LIMIT = 0.15;

    /*
        ######  #######    #     # ####### #######    ####### ######  ### #######
        #     # #     #    ##    # #     #    #       #       #     #  #     #
        #     # #     #    # #   # #     #    #       #       #     #  #     #
        #     # #     #    #  #  # #     #    #       #####   #     #  #     #
        #     # #     #    #   # # #     #    #       #       #     #  #     #
        #     # #     #    #    ## #     #    #       #       #     #  #     #
        ######  #######    #     # #######    #       ####### ######  ###    #

        ######  ####### #       ####### #     #    ####### #     # ###  #####
        #     # #       #       #     # #  #  #       #    #     #  #  #     #
        #     # #       #       #     # #  #  #       #    #     #  #  #
        ######  #####   #       #     # #  #  #       #    #######  #   #####
        #     # #       #       #     # #  #  #       #    #     #  #        #
        #     # #       #       #     # #  #  #       #    #     #  #  #     #
        ######  ####### ####### #######  ## ##        #    #     # ###  #####

        #       ### #     # #######
        #        #  ##    # #
        #        #  # #   # #
        #        #  #  #  # #####
        #        #  #   # # #
        #        #  #    ## #
        ####### ### #     # #######

        (Unless you know what you are doing)

     */

    // VuForia global variables
    // Class Members
    private OpenGLMatrix lastLocation;
    private VuforiaLocalizer vuforia;
    private List<VuforiaTrackable> allTrackables;
    private VuforiaTrackables targetsSkyStone;
    @Getter TFObjectDetector tfod;

    // To get this to work, copy the file VuForiaKey.java.java.example to VuForiaKey.java.java and add your key in that file.
    //private static final String VUFORIA_KEY = VuForiaKey.java.VUFORIAKEY;

    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch; // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23f   * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59; // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // VuForia object detection
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private boolean targetVisible;

    // TODO: Add more sensor capability
    public static HashMap<String, BNO055IMU> gyros; // Initialize gyroscopes
    public static HashMap<String, AnalogInput> pots; // Initialize potentiometers
    public static HashMap<String, DigitalChannel> buttons; // Initialize buttons
    public static HashMap<String, NormalizedColorSensor> colorSensors; // Initialize color sensors
    public static HashMap<String, DistanceSensor> distances; // Initialize distance sensors
    public static HashMap<String, WebcamName> webcams; // Initialize webcams
    public static HashMap<String, Encoder> encoders; // Special encoders

    public enum Colors {
        RED,
        ORANGE,
        YELLOW,
        GREEN,
        BLUE,
        PURPLE,
        WHITE,
        GRAY,
        BLACK,
        BROWN
    }

    /**
     * Gets the RGB value of the color sensor
     * @return enum for color
     */
    public Colors getRGB(String id, double redFudge, double greenFudge, double blueFudge) {
        float[] hsv = new float[3];
        Color.RGBToHSV(
                (int) Range.clip(getRed(id) * 255 * redFudge, 0, 255),
                (int) Range.clip(getGreen(id) * 255 * greenFudge, 0, 255),
                (int) Range.clip(getBlue(id) * 255 * blueFudge, 0, 255),
                hsv
        );

        if (hsv[1] < 0.2) {
            // Greyscale (inner core in HSV cylinder)
            if (hsv[2] > WHITE_THRESH) {
                return Colors.WHITE;
            } else if (hsv[2] < BLACK_THRESH) {
                return Colors.BLACK;
            }
        }

        // If the value is too low, black
        if (hsv[2] < BLACK_THRESH) {
            return Colors.BLACK;
        } else {
            if ((hsv[0] > 320) || (hsv[0] <= 20)) {
                return Colors.RED;
            } else if ((hsv[0] > 20) && (hsv[0] <= 46)) {
                return Colors.ORANGE;
            } else if ((hsv[0] > 46) && (hsv[0] <= 64)) {
                return Colors.YELLOW;
            } else if ((hsv[0] > 146) && (hsv[0] <= 160)) {
                return Colors.GREEN;
            } else if ((hsv[0] > 160) && (hsv[0] <= 248)) {
                return Colors.BLUE;
            } else if ((hsv[0] > 248) && (hsv[0] <= 320)) {
                return Colors.PURPLE;
            }
        }
        return Colors.GRAY;
    }

    public static BNO055IMU getGyro(String id) {
        return Objects.requireNonNull(gyros.get(id));
    }

    /**
     * Gets if a color sensor detects red
     * @param id ID of the color sensor
     * @return Boolean on if the sensor detects red or not
     */
    public double getRed(String id) {
        return Objects.requireNonNull(colorSensors.get(id)).getNormalizedColors().red;
    }

    /**
     * Gets if a color sensor detects green
     * @param id ID of the color sensor
     * @return Boolean on if the sensor detects green or not
     */
    public double getGreen(String id) {
        return Objects.requireNonNull(colorSensors.get(id)).getNormalizedColors().green;
    }

    /**
     * Gets if a color sensor detects blue
     * @param id ID of the color sensor
     * @return Boolean on if the sensor detects blue or not
     */
    public double getBlue(String id) {
        return Objects.requireNonNull(colorSensors.get(id)).getNormalizedColors().blue;
    }

    /**
     * Gets if a button is pressed
     * @param id ID of the button
     * @return Boolean of if the button is pressed or not
     */
    public boolean getButton(String id) {
        return !(Objects.requireNonNull(buttons.get(id)).getState());
    }

    /**
     * Gets the position (in degrees) of a potentiometer
     * @param id ID of the potentiometer
     * @return Degrees of the potentiometer
     */
    public double getPotDeg(String id) {
        return (POT_MAX/(Vmax-Vmin))*(Objects.requireNonNull(pots.get(id)).getVoltage()-Vmin); // Converts voltage to angle (degrees)
    }

    public static Encoder getEncoder(String id) {
        return encoders.get(id);
    }

    /**
     * Initializes the gyroscope.
     * @param id ID of the gyroscope
     */
    public void initGyro(String id) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = String.format(Locale.ENGLISH, "BNO055IMUCalibration%s.json", id);
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        Objects.requireNonNull(gyros.get(id)).initialize(parameters);
    }

    /**
     * Calibrates a gyroscope
     * @param id ID of the gyroscope to calibrate
     */
    public void calibrateGyro(String id) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        Objects.requireNonNull(gyros.get(id)).initialize(parameters);

        BNO055IMU.CalibrationData calibrationData = Objects.requireNonNull(gyros.get(id)).readCalibrationData();
        String filename = String.format(Locale.ENGLISH, "BNO055IMUCalibration%s.json", id);
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
    }

    public static double[] RGBtoHSV(double[] rgb) {
        double[] hsv = new double[] {0, 0, 0};
        double max = Math.max(Math.max(rgb[0], rgb[1]), rgb[2]);
        double min = Math.min(Math.min(rgb[0], rgb[1]), rgb[2]);
        double delta = max - min;

        if (delta == 0) {
            hsv[0] = 360;
            hsv[1] = 0;
            hsv[2] = max;
            return hsv;
        }

        if (max == rgb[0]) {
            hsv[0] = (rgb[1] - rgb[2]) / delta % 6;
        } else if (max == rgb[1]) {
            hsv[0] = (rgb[2] - rgb[0]) / delta + 2;
        } else {
            hsv[0] = (rgb[0] - rgb[1]) / delta + 4;
        }
        hsv[0] *= 60;

        if (max == 0) {
            hsv[1] = 0;
        } else {
            hsv[1] = delta / max;
        }

        hsv[2] = max;

        return hsv;
    }
}

