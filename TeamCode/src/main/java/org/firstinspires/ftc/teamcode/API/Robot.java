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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.API.Config.Constants;
import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.HW.SmartMotor;

import lombok.Builder;

/**
 * Integrates Sensor class and Movement class so we can use VuForia and motors in one function.
 * Use like so: Robot robot = new Robot(new Sensor(whatever), new Movement(whatever));
 * Refer to Movement and Sensor classes for more information on what those classes do.
 * NOTE: You really should edit this file to suit your robot. If you find an error occurring here,
 * add it to our GitHub issues page at https://github.com/botsburgh/BOTSBURGH-FTC-2020-21/issues
 */
@Builder
public class Robot {
    public static Sensor sensor;
    public static Movement movement;
    public static StateMachine state;
    public static LinearOpMode linearOpMode;
    public static void whiteLine(String sensor, double power) {
        driveToColor(sensor, power, Sensor.Colors.WHITE);
    }
    
    public static void driveToColor(String sensor, double power, Sensor.Colors targetColor) {
        while (true) {
            Sensor.Colors color = Sensor.getRGB(sensor);
            if (color == targetColor) {
                movement.move1x4(0);
                break;
            } else {
                movement.move1x4(power);
            }
        }
    }
    
    public static void shootAuto(int count, double volts) {
        // Power up flywheel
        movement.moveFlywheel(volts/Robot.sensor.getBatteryVoltage(linearOpMode.hardwareMap.voltageSensor));
        linearOpMode.sleep(500);
        for (int i =  0; i < count; i++) {
            movement.launch(true);
            linearOpMode.sleep(500);
            movement.launch(false);
        }
    }
    
    public static void shootAuto(int count) {
        shootAuto(count, 10.00);
    }
    
    public static void moveArm(String sensor, String arm) {
        SmartMotor armMotor = Movement.getMotor(arm);
        Sensor.Colors target;

        if (state.getCurrentState() == StateMachine.State.ARMOUT) {
            armMotor.setDirection(DcMotor.Direction.FORWARD);
            target = Constants.ARM_EXTEND_COLOR;
        } else if (state.getCurrentState() == StateMachine.State.ARMIN) {
            armMotor.setDirection(DcMotor.Direction.REVERSE);
            target = Constants.ARM_CLOSE_COLOR;
        } else {
            target = null;
        }
        
        long end = System.currentTimeMillis() + 5000;
        
        if (
                ((state.getCurrentState() == StateMachine.State.ARMIN) ||
                        (state.getCurrentState() == StateMachine.State.ARMOUT)) &&
                (Sensor.getRGB(sensor) != target) &&
                (!linearOpMode.isStopRequested()) &&
                (System.currentTimeMillis() < end)
        ) {
            armMotor.setPower(Constants.MOTOR_ARM_POWER);
        } else {
            state.setCurrentState(StateMachine.State.DRIVE);
            armMotor.setPower(0);
        }
    }

    public static void moveArm(boolean command, String sensor, String arm) {
        SmartMotor armMotor = Movement.getMotor(arm);
        Sensor.Colors target;

        if (command) {
            armMotor.setDirection(DcMotor.Direction.FORWARD);
            target = Constants.ARM_EXTEND_COLOR;
        } else {
            armMotor.setDirection(DcMotor.Direction.REVERSE);
            target = Constants.ARM_CLOSE_COLOR;
        }

        long end = System.currentTimeMillis() + 5000;

        while (
                    (Sensor.getRGB(sensor) != target) &&
                    (!linearOpMode.isStopRequested()) &&
                    (System.currentTimeMillis() < end)
        ) {
            armMotor.setPower(Constants.MOTOR_ARM_POWER);
        }

        armMotor.setPower(0);
    }


    public static void wobbleDrop() {
        Robot.moveArm(true, Naming.COLOR_SENSOR_ARM, Naming.MOTOR_WOBBLE_ARM);
        Robot.movement.grabWobble(true);
        linearOpMode.sleep(500);
        Robot.moveArm(false, Naming.COLOR_SENSOR_ARM, Naming.MOTOR_WOBBLE_ARM);
    }
}