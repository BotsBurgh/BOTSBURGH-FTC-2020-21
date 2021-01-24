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

import org.firstinspires.ftc.teamcode.API.InitRobot;
import org.firstinspires.ftc.teamcode.API.Robot;

@TeleOp(name = "TeleOp Main", group = "Linear OpMode")
public class TeleOpMain extends LinearOpMode {
    private double maxspeed = 0.6;
    private boolean switch = false;
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
            Robot.movement.moveFlywheel(gamepad2.left_trigger);
            Robot.movement.moveIntake(gamepad2.left_trigger);
            
            if (gamepad1.left_bumper) {
                if (switch) {
                    Robot.movement.getMotor("fl").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Robot.movement.getMotor("fr").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Robot.movement.getMotor("bl").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Robot.movement.getMotor("br").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } else {
                    Robot.movement.getMotor("fl").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    Robot.movement.getMotor("fr").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    Robot.movement.getMotor("bl").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    Robot.movement.getMotor("br").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                switch = !switch;
            }
            
            if (gamepad1.right_bumper) {
                maxspeed = 0.8;
            } else {
                maxspeed = 0.6;
            }

            // Moving the wobble
            if (gamepad2.x) {
                Robot.movement.moveWobble(true);
            } else if (gamepad2.y) {
                Robot.movement.moveWobble(false);
            }
            
            if (gamepad2.a) {
                Robot.movement.grabWobble(true);
            } else if (gamepad2.b) {
                Robot.movement.grabWobble(false);
            }

            // Launch async because we don't want the robot to hang while it does stuff
            Robot.movement.launch(gamepad2.left_bumper);

            telemetry.addData("Back Left", blPower);
            telemetry.addData("Back Right", brPower);
            telemetry.addData("Front Left", flPower);
            telemetry.addData("Front Right", frPower);
            telemetry.update();
        }
    }
}
