package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RedShootPath extends LinearOpMode {
    @Override public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-37, -43, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-24, -59, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-7, -32, Math.toRadians(180)), Math.toRadians(90))
                .build();

        drive.followTrajectory(trajectory);
    }
}
