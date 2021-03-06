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

package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.InitRobot;
import org.firstinspires.ftc.teamcode.API.Movement;
import org.firstinspires.ftc.teamcode.API.Robot;
import org.firstinspires.ftc.teamcode.API.Sensor;
import org.firstinspires.ftc.teamcode.API.StateMachine;

@TeleOp(name = "TeleOp Main", group = "00-teleop")
public class TeleOpMain extends LinearOpMode {
    private double maxspeed = 0.6;
    private boolean brakeSwitch = false;
    @Override
    public void runOpMode() {
        InitRobot.init(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        telemetry.addData(">", "Press stop");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            double flPower = Range.clip(( x1 - y1 + rotation), -maxspeed, maxspeed);
            double blPower = Range.clip((-x1 - y1 + rotation), -maxspeed, maxspeed);
            double brPower = Range.clip(( x1 - y1 - rotation), -maxspeed, maxspeed);
            double frPower = Range.clip((-x1 - y1 - rotation), -maxspeed, maxspeed);

            Robot.movement.move4x4(flPower, frPower, blPower, brPower);
            if (gamepad2.left_trigger < 0.1) {
                Robot.movement.moveFlywheel(0);
            } else if (gamepad2.left_trigger < 0.9) {
                Robot.movement.moveFlywheel(10.0/Robot.sensor.getBatteryVoltage(hardwareMap.voltageSensor));
            } else if (gamepad2.left_trigger <= 1) {
                Robot.movement.moveFlywheel(11.27/Robot.sensor.getBatteryVoltage(hardwareMap.voltageSensor));
            } else {
                Robot.movement.moveFlywheel(0);
            }
            Robot.movement.moveIntake(gamepad2.right_trigger);
            Robot.movement.moveIntake((gamepad2.left_bumper ? -1 : 0));
            
            if (gamepad1.left_bumper) {
                if (brakeSwitch) {
                    Movement.getMotor("fl").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Movement.getMotor("fr").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Movement.getMotor("bl").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Movement.getMotor("br").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } else {
                    Movement.getMotor("fl").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    Movement.getMotor("fr").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    Movement.getMotor("bl").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    Movement.getMotor("br").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                brakeSwitch = !brakeSwitch;
            }
            
            if (gamepad1.right_bumper) {
                maxspeed = 0.8;
            } else {
                maxspeed = 0.6;
            }

            // Moving the wobble
            if (gamepad2.x) {
                if (Sensor.getRGB(Naming.COLOR_SENSOR_ARM) != Sensor.Colors.RED) {
                    Robot.state.setCurrentState(StateMachine.State.ARMOUT);
                }
            } else if (gamepad2.y) {
                if (Sensor.getRGB(Naming.COLOR_SENSOR_ARM) != Sensor.Colors.BLUE) {
                    Robot.state.setCurrentState(StateMachine.State.ARMIN);
                }
            }
            Robot.moveArm(Naming.COLOR_SENSOR_ARM, Naming.MOTOR_WOBBLE_ARM);

            if (gamepad2.a) {
                Robot.movement.grabWobble(true);
            } else if (gamepad2.b) {
                Robot.movement.grabWobble(false);
            }

            Robot.movement.launch(gamepad2.right_bumper);

            telemetry.addData("Back Left", blPower);
            telemetry.addData("Back Right", brPower);
            telemetry.addData("Front Left", flPower);
            telemetry.addData("Front Right", frPower);
            telemetry.addData("Flywheel", Movement.getMotor(Naming.MOTOR_FLYWHEEL).getPower());
            telemetry.addData("Wobble Arm", Movement.getMotor(Naming.MOTOR_WOBBLE_ARM).getCurrentPosition());
            telemetry.update();
        }
    }
}
