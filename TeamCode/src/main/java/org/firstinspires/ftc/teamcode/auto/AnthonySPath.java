package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AnthonySPath extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-50,-50, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(-25, -60), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-5, -37.5), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(-5, -37.5), () -> {
                    // shoot
                })
                .splineToLinearHeading(new Pose2d(37.5, -37.5), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-25, -12.5), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-50, -12.5), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-25, -12.5), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(37.5, -37.5), Math.toRadians(0))
                .build();
        drive.followTrajectory(trajectory);
    }
}
