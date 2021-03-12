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
    public static double DRIVEFUDGE = 60.0/42;
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

        // Go to the white line
        Robot.whiteLine(Naming.COLOR_SENSOR_PARK, 0.4);

        // Move so we are in the launch zone
        Trajectory moveback = drive.trajectoryBuilder(new Pose2d())
                .back(DRIVEY*DRIVEFUDGE)
                .build();
        drive.followTrajectory(moveback);

        drive.turn(-0.1*TURNFUDGE);

        // Shoot the first disk
        Robot.shootAuto(Naming.MOTOR_FLYWHEEL, Naming.SERVO_LAUNCHER, 0.8, 1);
        drive.turn(-0.3*TURNFUDGE);
        // Shoot again
        Robot.shootAuto(Naming.MOTOR_FLYWHEEL, Naming.SERVO_LAUNCHER, 0.8, 1);
        drive.turn(-0.2*TURNFUDGE);
        // And again
        Robot.shootAuto(Naming.MOTOR_FLYWHEEL, Naming.SERVO_LAUNCHER, 0.8, 1);
        //drive.turn(0.3*TURNFUDGE);
        // Go back to the white line and park
        Robot.whiteLine(Naming.COLOR_SENSOR_PARK, 0.4);
    }
}