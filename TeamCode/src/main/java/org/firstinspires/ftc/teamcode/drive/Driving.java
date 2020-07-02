package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robot;

import android.os.AsyncTask;


@Config
@TeleOp (name = "Driving", group = "00-TeleOp")
public class Driving extends LinearOpMode{

    Robot robot = new Robot(hardwareMap);

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Intialized");
        telemetry.update();

        while (opModeIsActive()) {

            // Android phone display information
            telemetry.addData("Status", "Run Time: " + getRuntime());
            telemetry.update();
        }
    }

    // TODO: Work on reimplementing
    private class AsyncBase extends AsyncTask<Robot, String, String> {

        @Override
        protected String doInBackground(Robot... params) {

            while (opModeIsActive()) {
                Pose2d baseVel = new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                );

                Pose2d vel;
                if (Math.abs(baseVel.getX()) + Math.abs(baseVel.getY()) + Math.abs(baseVel.getHeading()) > 1) {
                    // re-normalize the powers according to the weights
                    double denom = VX_WEIGHT * Math.abs(baseVel.getX())
                            + VY_WEIGHT * Math.abs(baseVel.getY())
                            + OMEGA_WEIGHT * Math.abs(baseVel.getHeading());
                    vel = new Pose2d(
                            VX_WEIGHT * baseVel.getX(),
                            VY_WEIGHT * baseVel.getY(),
                            OMEGA_WEIGHT * baseVel.getHeading()
                    ).div(denom);
                } else {
                    vel = baseVel;
                }

                robot.setDrivePower(vel);

                robot.update();

                Pose2d poseEstimate = robot.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.update();

            }
            return null;
        }
    }
}
