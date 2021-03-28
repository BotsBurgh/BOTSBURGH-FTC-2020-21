package org.firstinspires.ftc.teamcode.OpMode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.API.InitRobot;
import org.firstinspires.ftc.teamcode.API.Robot;
import org.firstinspires.ftc.teamcode.API.SampleMecanumDrive;

@Config
@Autonomous(name="Autonomous Shooter", group = "00-drive")
public class AutonomousShooter extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        InitRobot.init(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // All Trajectories
        Trajectory forwardTrajectory =  drive.trajectoryBuilder(new Pose2d())
                .forward(60)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Actual autonomous movement
        drive.followTrajectory(forwardTrajectory);
        Robot.movement.moveFlywheel(1);
        sleep(3000);
        Robot.movement.launch(true);
        sleep(1000);
        Robot.movement.launch(false);
        Robot.movement.moveFlywheel(0);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}

