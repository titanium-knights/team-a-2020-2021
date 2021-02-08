package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;

public class JeffreyPath extends LinearOpMode {
    @Override public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-60,-25, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(0,-36, Math.toRadians(0)), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(0,-36), () -> {
                    //shoot
                })
                .splineToLinearHeading(new Pose2d(35,-25, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(10,-12, Math.toRadians(0)), Math.toRadians(0))
                .build();
        drive.setPoseEstimate(trajectory.start());
        drive.followTrajectory(trajectory);
    }
}