package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class roadRunnerthing extends LinearOpMode {
    @Override public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-63, -38));
            .spline trajectory = drive.trajectoryBuilder(new Pose2d(15,-38));
            .spline trajectory = drive.trajectoryBuilder(new Pose2d(15,-15));
            .spline trajectory = drive.trajectoryBuilder(new Pose2d(40,-38));
            .spline trajectory = drive.trajectoryBuilder(new Pose2d(-63,-38));
            .spline trajectory = drive.trajectoryBuilder(new Pose2d(40,-38));
        }
    }
}
