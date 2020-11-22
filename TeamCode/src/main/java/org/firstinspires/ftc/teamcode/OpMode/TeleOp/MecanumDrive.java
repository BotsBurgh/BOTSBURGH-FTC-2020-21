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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.API.InitRobot;
import org.firstinspires.ftc.teamcode.API.Robot;

@TeleOp(name = "Mecanum Drive", group = "Linear OpMode")
public class MecanumDrive extends LinearOpMode {
    private static final double MAX_SPEED = 0.8;
    @Override
    public void runOpMode() {
        InitRobot.init(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        telemetry.addData(">", "Press stop");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            double x1 = -gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double rotation = -gamepad1.right_stick_x;

            double flPower = Range.clip((y1 + x1 + rotation), -MAX_SPEED, MAX_SPEED);
            double frPower = Range.clip((y1 - x1 - rotation), -MAX_SPEED, MAX_SPEED);
            double blPower = Range.clip((y1 - x1 + rotation), -MAX_SPEED, MAX_SPEED);
            double brPower = Range.clip((y1 + x1 - rotation), -MAX_SPEED, MAX_SPEED);

            Robot.movement.move4x4(flPower, frPower, blPower, brPower);

            telemetry.addData("Back Left", blPower);
            telemetry.addData("Back Right", brPower);
            telemetry.addData("Front Left", flPower);
            telemetry.addData("Front right", frPower);
            telemetry.update();
        }
    }
}