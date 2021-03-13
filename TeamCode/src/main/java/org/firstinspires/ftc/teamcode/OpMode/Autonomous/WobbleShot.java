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
    public static double DRIVEFUDGE = 60.0/42;
    public static double DRIVEY     = 15;
    public static double TURNFUDGE  = 180.0/150;
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

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeRight(45*DRIVEFUDGE).build()); // Right 10

        // Go to the white line
        Robot.whiteLine(Naming.COLOR_SENSOR_PARK, 0.4);
        
        if (disks == Sensor.Disks.ONE) {
            drive.turn(0.4*TURNFUDGE);
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).forward(20*DRIVEFUDGE).build()); // Forward 20
        } else if (disks == Sensor.Disks.FOUR) {
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).forward(400*DRIVEFUDGE).build()); // Forward 40
        } else {
            drive.turn(-100*TURNFUDGE);
        }
        
        Robot.moveArm(true, Naming.COLOR_SENSOR_ARM, Naming.MOTOR_WOBBLE_ARM);
        Robot.movement.grabWobble(false);
        sleep(500);
        Robot.moveArm(false, Naming.COLOR_SENSOR_ARM, Naming.MOTOR_WOBBLE_ARM);

        if (disks == Sensor.Disks.ONE) {
            drive.turn(-0.8*TURNFUDGE);
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).forward(-25*DRIVEFUDGE).build()); // Backward 20
            drive.turn(0.4*TURNFUDGE);
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeRight(10*DRIVEFUDGE).build()); // Left 10
        } else if (disks == Sensor.Disks.FOUR) {
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).forward(-400*DRIVEFUDGE).build()); // Backward 20
            drive.turn(100*TURNFUDGE);
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).forward(-400*DRIVEFUDGE).build()); // Backward 20
        } else {
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).back(-24*DRIVEFUDGE).build()); // Backward 24
            drive.turn(100*TURNFUDGE);
        }

        // Move so we are in the launch zone
        Trajectory moveback = drive.trajectoryBuilder(new Pose2d())
                .back(DRIVEY*DRIVEFUDGE)
                .build();
        drive.followTrajectory(moveback);

        drive.turn(-0.1*TURNFUDGE);

        // Shoot the first disk
        Robot.shootAuto(1);
        drive.turn(-0.3*TURNFUDGE);
        // Shoot again
        Robot.shootAuto(1);
        drive.turn(-0.2*TURNFUDGE);
        // And again
        Robot.shootAuto(1);
        //drive.turn(0.3*TURNFUDGE);
        // Go back to the white line and park
        Robot.whiteLine(Naming.COLOR_SENSOR_PARK, 0.4);
    }
}