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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import lombok.Builder;
import lombok.Getter;

/**
 * Integrates Sensor class and Movement class so we can use VuForia and motors in one function.
 * Use like so: Robot robot = new Robot(new Sensor(whatever), new Movement(whatever));
 * Refer to Movement and Sensor classes for more information on what those classes do.
 * NOTE: You really should edit this file to suit your robot. If you find an error occurring here,
 * add it to our GitHub issues page at https://github.com/botsburgh/BOTSBURGH-FTC-2019-20/issues
 */
@Builder
public class Robot {
    @Getter private Sensor sensor;
    @Getter private Movement movement;

    private static final double COUNTS_PER_MOTOR_REV  = 1440; // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION  = 1.0;  // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;  // For figuring circumference
    private static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double HEADING_THRESHOLD = 2.0;  // Any less and our robot acts up
    private static final double P_TURN_COEFF      = 0.1;  // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF     = 0.15; // Larger is more responsive, but also less stable

    private static final double TURN_TIMEOUT      = 4.0;  // Timeout to prevent non-stop turning
    private static final double DRIVE_TIMEOUT     = 10.0; // Maximum execution time for driving

    // Quick and dirty hack to prevent issues with stopping the robot
    @Getter
    private LinearOpMode linearOpMode;
}
