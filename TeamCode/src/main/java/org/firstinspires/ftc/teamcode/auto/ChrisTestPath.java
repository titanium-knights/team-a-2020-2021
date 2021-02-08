package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RedShootPath extends LinearOpMode {
    @Override public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-63, -38, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(15, -38, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(15, -13, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-63, -13, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(15, -13, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(15, -38, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(10, 0, Math.toRadians(0)), Math.toRadians(0))
                .build();
        
        drive.followTrajectory(trajectory);
    }
}
