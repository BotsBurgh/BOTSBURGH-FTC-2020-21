/*
Copyright 2020 FIRST Tech Challenge Team 11792
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode.API;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.HW.SmartMotor;
import org.firstinspires.ftc.teamcode.API.HW.SmartServo;

import java.util.HashMap;
import java.util.Objects;

import lombok.Builder;

/**
 * The Movement class. Interfaces with servos and motors so you don't have to
 */
@Builder
public class Movement {

    // Autonomous
    final double DRIVE_POWER = 0.6; // How fast to drive

    // Elevator configuration
    private final static double ELEVATOR_POWER   = 1.00;
    
    // Motor configuration
    private final static double INTAKE2_THRESH   = 1.0;

    // Servo configuration
    private final static int    SERVO_SLEEP      = 10; // Milliseconds
    private final static double SERVO_STEP       = 0.01;  // on a scale of  0-1
    private final static double WOBBLE_GRAB      = 0.3;
    private final static double WOBBLE_RELEASE   = 0;
    private final static double LAUNCHER_OPEN    = 0.6;
    private final static double LAUNCHER_CLOSE   = 1;

    public static HashMap<String, SmartMotor> motors;
    public static HashMap<String, SmartServo> servos;
    public static HashMap <String, CRServo> crServos;

    // Getters

    public static SmartMotor getMotor(String id) {
        return motors.get(id);
    }

    public SmartServo getServo(String id) {
        return servos.get(id);
    }

    public CRServo getCRServo(String id) {
        return crServos.get(id);
    }

    /**
     * Moves each of the four motors individually. Best for mecanum drives.
     * @param flPower Power to the front left wheel
     * @param frPower Power to the front right wheel
     * @param blPower Power to the back left wheel
     * @param brPower Power to the back right wheel
     */
    public void move4x4(double flPower, double frPower, double blPower, double brPower) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_FL)).setPower(flPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_FR)).setPower(frPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BL)).setPower(blPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BR)).setPower(brPower);
    }

    /**
     * Moves each of the four motors individually. Best for mecanum drives.
     * @param power Power to all the wheels
     */
    public void move1x4(double power) {
        move4x4(power,power,power,power);
    }


    /**
     * Move the base with four motors based on individual sides, not wheels
     * @param lPower Power to the left side
     * @param rPower Power to the right side
     */
    public void move2x4(double lPower, double rPower) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_FL)).setPower(lPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_FR)).setPower(rPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BL)).setPower(lPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BR)).setPower(rPower);
    }

    /**
     * Move the base with two motors and two values
     * @param lPower Power sent to back left motor
     * @param rPower Power sent to back right motor
     */
    public void move2x2(double lPower, double rPower) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_BL)).setPower(lPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BR)).setPower(rPower);
    }

    /**
     * Moves the lift up and down, depending on the power sent to the motor. Subject to threshold
     * @param speed Speed of the elevator
     */
    public void moveElevator(double speed) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_LIFT)).setPower(speed*ELEVATOR_POWER);
    }

    /**
     * Sets the servo to a specific position. Useful if we do not want to slowly scan the servo to a position
     * @param id ID of the servo
     * @param degrees Position (in on a scale of  0-1) to set the servo to.
     */
    public void setServo(String id, double degrees) {
        Objects.requireNonNull(servos.get(id)).setPosition(degrees);
    }

    /**
     * Scan the servo (move the servo slowly) to a position.
     * @param id ID of servo
     * @param degrees Position (in on a scale of  0-1) to scan the servo to.
     */
    public void scanServo(String id, double degrees, boolean clockwise) {
        while (Math.abs(Objects.requireNonNull(servos.get(id)).getPosition() - degrees) < 0.001) {
            if (clockwise) {
                // Scan down
                Objects.requireNonNull(servos.get(id)).setPosition(Objects.requireNonNull(servos.get(id)).getPosition() - SERVO_STEP);
            } else {
                // Scan up
                Objects.requireNonNull(servos.get(id)).setPosition(Objects.requireNonNull(servos.get(id)).getPosition() + SERVO_STEP);
            }
        }
    }

    /**
     * Set the speed of a continuous rotation servo
     * @param id ID of CRServo
     * @param power Power (and subsequently speed) sent to CRServo
     */
    public void setServoSpeed(String id, double power) {
        Objects.requireNonNull(crServos.get(id)).setPower(power);
    }

    /**
     * Set the movement of the flywheel
     * @param wheelPower Power sent to flywheel
     */
    public void moveFlywheel(double wheelPower) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_FLYWHEEL)).setPower(wheelPower);
    }

    /**
     * Set the movement of the intake
     * @param intakePower Power sent to intake
     */
    public void moveIntake(double intakePower) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_INTAKE)).setPower(intakePower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_INTAKE2)).setPower(intakePower*INTAKE2_THRESH);
    }

    /**
     * Opens the grabber based on a boolean assignment
     * @param command tue to open
     */
    public void grabWobble(boolean command) {
        if (command) {
            Objects.requireNonNull(servos.get(Naming.SERVO_WOBBLE_GRABBER)).setPosition(WOBBLE_GRAB); // Opens the grabber
        } else {
            Objects.requireNonNull(servos.get(Naming.SERVO_WOBBLE_GRABBER)).setPosition(WOBBLE_RELEASE); // Closes the grabber
        }
    }


    public void launch(boolean command) {
        if (command) {
            Objects.requireNonNull(servos.get(Naming.SERVO_LAUNCHER)).setPosition(LAUNCHER_OPEN);
        } else {
            Objects.requireNonNull(servos.get(Naming.SERVO_LAUNCHER)).setPosition(LAUNCHER_CLOSE);
        }
    }
}
