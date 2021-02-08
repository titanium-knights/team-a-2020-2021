package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class VedicTestPath extends LinearOpMode {
    @Override public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(-63, -26, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-40, -37, Math.toRadians(0)), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(-40, -37), () -> {
                    // Shoot
                })
                .addTemporalMarker(2, () -> {

                })
                .splineToLinearHeading(new Pose2d(-22, -37, Math.toRadians(0)), Math.toRadians(0))
                .build();

        drive.setPoseEstimate(trajectory.start());
        drive.followTrajectory(trajectory);
    }
}
