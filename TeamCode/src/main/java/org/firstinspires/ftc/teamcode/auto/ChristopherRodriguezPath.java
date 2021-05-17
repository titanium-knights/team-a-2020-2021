package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class ChristopherRodriguezPath extends LinearOpMode {
    @Override public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-60,25, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(10,25, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50,60, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-49,50, Math.toRadians(0)), Math.toRadians(0))
                .build();
        drive.setPoseEstimate(trajectory.start());
        drive.followTrajectory(trajectory);
    }
}
