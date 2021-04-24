package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.InitRobot;
import org.firstinspires.ftc.teamcode.API.Robot;
import org.firstinspires.ftc.teamcode.API.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.API.Sensor;

/*
 * This is an autonomous program to shoot the power shots, drop off the wobble goals, and park
 */
@Config
@Autonomous(name="Wobble RR Shot", group = "00-drive")
public class WobbleRRShot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        InitRobot.init(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Sensor.initVuforia(Naming.WEBCAM_0);
        Sensor.initTfod();

        Pose2d position = new Pose2d();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine();
        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Sensor.Disks disks = Sensor.detectDisks();

        telemetry.addData("Detected:", disks);
        telemetry.update();
        // Go to the white line
        //Robot.whiteLine(Naming.COLOR_SENSOR_PARK, 0.4);
        drive.followTrajectory(drive.trajectoryBuilder(position).forward(81).build());

        drive.turn(0.2);

        // Shoot the first disk
        Robot.shootAuto( 1, 9.75);
        drive.turn(-0.25);
        // Shoot again
        Robot.shootAuto(1);
        drive.turn(-0.30);
        // And again
        Robot.shootAuto(1);
        // Go back to the white line
        Robot.whiteLine(Naming.COLOR_SENSOR_PARK, 0.4);
        Robot.movement.moveFlywheel(0);

        drive.turn(-Math.PI); // Roughly 90 degrees
        Robot.driveToColor(Naming.COLOR_SENSOR_PARK, 0.4, Sensor.Colors.RED);
        
        if (disks == Sensor.Disks.ONE) {
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeLeft(40).build());
            Robot.moveArm(true, Naming.COLOR_SENSOR_ARM, Naming.MOTOR_WOBBLE_ARM);
            Robot.wobbleDrop();
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeRight(40).build());
        } else if (disks == Sensor.Disks.NONE) {
            drive.turn(-Math.PI/2); // Roughly 90 degrees
            Robot.wobbleDrop();
        } else {
            Robot.movement.move1x4(-0.4);
            sleep(400);
            Robot.movement.move1x4(0);
            drive.turn(-Math.PI/2); // Roughly 90 degrees
            Robot.driveToColor(Naming.COLOR_SENSOR_PARK, -0.4, Sensor.Colors.RED);
            Robot.movement.move1x4(-0.4);
            sleep(600);
            Robot.movement.move1x4(0);
            Robot.driveToColor(Naming.COLOR_SENSOR_PARK, -0.4, Sensor.Colors.RED);
            Robot.wobbleDrop();
            Robot.whiteLine(Naming.COLOR_SENSOR_PARK, 0.4);
        }
    }
}