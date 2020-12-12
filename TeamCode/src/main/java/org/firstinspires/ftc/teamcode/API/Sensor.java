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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    private static final double RED_THRESH =   500;
    private static final double GREEN_THRESH = 700;
    private static final double BLUE_THRESH =  600;

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

    // To get this to work, copy the file VuForiaKey.java.example to VuForiaKey.java and add your key in that file.
    //private static final String VUFORIA_KEY = VuForiaKey.VUFORIAKEY;

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
    public static HashMap<String, ColorSensor> colorSensors; // Initialize color sensors
    public static HashMap<String, DistanceSensor> distances; // Initialize distance sensors
    public static HashMap<String, WebcamName> webcams; // Initialize webcams
    public static HashMap<String, Encoder> encoders; // Special encoders

    /**
     * Gets the RGB value of the color sensor
     * @return 0 if red, 1 if green, 2 if blue, 3 if none
     */
    public int getRGB(String id, double redThresh, double greenThresh, double blueThresh) {
        double red   = getRed(id);
        double green = getGreen(id);
        double blue  = getBlue(id);

        if (((Math.abs(red-green)/red) < THRESH_LIMIT) || ((Math.abs(red-green)/green) < THRESH_LIMIT) || ((Math.abs(red-blue)/red) < THRESH_LIMIT) ||
                ((Math.abs(red-blue)/blue) < THRESH_LIMIT) || ((Math.abs(green-blue)/green) < THRESH_LIMIT) || ((Math.abs(green-blue)/blue) < THRESH_LIMIT)) {
            return 3;
        }

        if ((red>blue) && (red>green) && (red>redThresh)) {
            return 0;
        } else if ((green>red) && (green>blue) && (green>greenThresh)) {
            return 1;
        } else if ((blue>red) && (blue>green) && (blue>blueThresh)) {
            return 2;
        } else {
            return 3;
        }
    }

    /**
     * Gets the RGB value of the color sensor
     * @return 0 if red, 1 if green, 2 if blue, 3 if none
     */
    public int getRGB(String id) {
        return getRGB(id, RED_THRESH, GREEN_THRESH, BLUE_THRESH);
    }

    public static BNO055IMU getGyro(String id) {
        return gyros.get(id);
    }

    /**
     * Gets if a color sensor detects red
     * @param id ID of the color sensor
     * @return Boolean on if the sensor detects red or not
     */
    public int getRed(String id) {
        return Objects.requireNonNull(colorSensors.get(id)).red();
    }

    /**
     * Gets if a color sensor detects green
     * @param id ID of the color sensor
     * @return Boolean on if the sensor detects green or not
     */
    public int getGreen(String id) {
        return Objects.requireNonNull(colorSensors.get(id)).green();
    }

    /**
     * Gets if a color sensor detects blue
     * @param id ID of the color sensor
     * @return Boolean on if the sensor detects blue or not
     */
    public int getBlue(String id) {
        return Objects.requireNonNull(colorSensors.get(id)).blue();
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
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
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
}

