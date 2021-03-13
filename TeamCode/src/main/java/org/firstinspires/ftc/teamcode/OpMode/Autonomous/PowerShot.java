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
import org.firstinspires.ftc.teamcode.R;

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

        // Shoot the first disk
        Robot.shootAuto( 1);
        drive.turn(-0.3*TURNFUDGE);
        // Shoot again
        Robot.shootAuto(1);
        drive.turn(0.5*TURNFUDGE);
        // And again
        Robot.movement.moveFlywheel(1); // We add vibrations
        sleep(2100); // And wait so the last wheel doesn't get stuck
        Robot.movement.moveFlywheel(0);
        sleep(500);
        Robot.shootAuto(1);
        //drive.turn(0.3*TURNFUDGE);
        // Go back to the white line and par
        Robot.whiteLine(Naming.COLOR_SENSOR_PARK, 0.4);
    }
}