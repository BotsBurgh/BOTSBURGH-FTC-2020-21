package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.InitRobot;
import org.firstinspires.ftc.teamcode.API.Movement;
import org.firstinspires.ftc.teamcode.API.Robot;
import org.firstinspires.ftc.teamcode.API.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.API.Sensor;

/*
 * This is a simple program to reach the white line
 */
@Config
@Autonomous(name="Wobble Shot", group = "drive")
public class WobbleShot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        InitRobot.init(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Sensor.initVuforia(Naming.WEBCAM_0);
        Sensor.initTfod();


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
        Robot.whiteLine(Naming.COLOR_SENSOR_PARK, 0.4);

        // Move back so we are in the launch zone
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(7).build());

        // Get in position to shoot...
        drive.turn(Math.toRadians(2));
        // Shoot the first disk
        Robot.shootAuto( 1);
        // Adjust and shoot
        drive.turn(Math.toRadians(-7));
        Robot.shootAuto(1);
        // And again
        drive.turn(Math.toRadians(-5));
        Robot.shootAuto(1, 9.75);
        // Make sure that the flywheel is unpowered
        Robot.movement.moveFlywheel(0);

        // Go back to the white line and park
        Robot.whiteLine(Naming.COLOR_SENSOR_PARK, 0.4);

        drive.turn(Math.toRadians(-80));
        Robot.driveToColor(Naming.COLOR_SENSOR_PARK, 0.4, Sensor.Colors.RED);
        
        if (disks == Sensor.Disks.ONE) {
            Robot.movement.move4x4(-0.4, 0.4, 0.4, -0.4);
            sleep(750);
            Robot.movement.move1x4(0);
            Robot.wobbleDrop();
            Robot.movement.move4x4(0.4, -0.4, -0.4, 0.4);
            sleep(750);
            Robot.movement.move1x4(0);
        } else if (disks == Sensor.Disks.NONE) {
            drive.turn(Math.toRadians(-90)); // Roughly 90 degrees
            Robot.wobbleDrop();
        } else {
            Robot.movement.move1x4(-0.4);
            sleep(200);
            Robot.movement.move1x4(0);
            drive.turn(Math.toRadians(-90)); // Roughly 90 degrees
            Robot.driveToColor(Naming.COLOR_SENSOR_PARK, -0.4, Sensor.Colors.RED);
            Robot.movement.move1x4(-0.4);
            sleep(500);
            Robot.movement.move1x4(0);
            Robot.driveToColor(Naming.COLOR_SENSOR_PARK, -0.4, Sensor.Colors.RED);
            Robot.wobbleDrop();
            Robot.whiteLine(Naming.COLOR_SENSOR_PARK, 0.4);
        }
    }
}