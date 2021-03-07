package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.InitRobot;
import org.firstinspires.ftc.teamcode.API.Robot;
import org.firstinspires.ftc.teamcode.API.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.API.Sensor;

/*
 * This is a simple program to reach the white line
 */
@Config
@Autonomous(name="Power Shot", group = "drive")
public class PowerShot extends LinearOpMode {
    public static double FUDGE = 10;
    public static double REDFUDGE   = 25*FUDGE;
    public static double GREENFUDGE = 15*FUDGE;
    public static double BLUEFUDGE  = 15*FUDGE;
    public static double DRIVEFUDGE = 60.0/42;
    public static double DRIVEX     = 15;
    public static double DRIVEY     = 15;
    public static double TURNFUDGE  = 180.0/150;
    @Override
    public void runOpMode() throws InterruptedException {
        InitRobot.init(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine();
        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Move away from the wall
        //Robot.movement.move4x4(0.3,0.3,0.3,0.3);
        //sleep(500);
        //Robot.movement.move4x4(0,0,0,0);

        // Move right a little so we are aligned with the power shots
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(DRIVEX*DRIVEFUDGE) // we tell it to go left, but it goes right. whatever
                .build();

        //drive.followTrajectory(traj);

        // Go to the white line
        Robot.whiteLine(REDFUDGE, GREENFUDGE, BLUEFUDGE, Naming.COLOR_SENSOR_PARK, 0.4);

        // Move so we are in the launch zone
        Trajectory moveback = drive.trajectoryBuilder(new Pose2d())
                .back(DRIVEY*DRIVEFUDGE)
                .build();
        drive.followTrajectory(moveback);

        drive.turn(-0.1*TURNFUDGE);

        // Shoot the first disk
        Robot.shootAuto(Naming.MOTOR_FLYWHEEL, Naming.SERVO_LAUNCHER, 0.8, 1);
        drive.turn(-0.35*TURNFUDGE);
        // Shoot again
        Robot.shootAuto(Naming.MOTOR_FLYWHEEL, Naming.SERVO_LAUNCHER, 0.8, 1);
        drive.turn(-0.3*TURNFUDGE);
        // And again
        Robot.shootAuto(Naming.MOTOR_FLYWHEEL, Naming.SERVO_LAUNCHER, 0.8, 1);
        //drive.turn(0.3*TURNFUDGE);
        // Go back to the white line and park
        Robot.whiteLine(REDFUDGE, GREENFUDGE, BLUEFUDGE, Naming.COLOR_SENSOR_PARK, 0.4);
    }
}