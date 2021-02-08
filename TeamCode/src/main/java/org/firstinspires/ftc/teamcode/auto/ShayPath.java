package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class ShayPath extends LinearOpMode {
    @Override public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-64, -26, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(-24, -26, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24, -37, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(54, -37, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(60, -53, Math.toRadians(0)), Math.toRadians(0))
                .build();

        drive.followTrajectory(trajectory);
    }
}
