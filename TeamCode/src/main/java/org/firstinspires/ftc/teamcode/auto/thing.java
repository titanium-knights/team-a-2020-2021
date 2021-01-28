package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class thing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        TrajectoryBuilder builder = drive.trajectoryBuilder(new Pos2d());
        builder.lineTo(new Vector2D(10, 10));
        builder.addSpacialMarker(new Vector2D(10,10), () -> {
            shooter.toggleShooterPower();
            shooter.nudgeRings(true, false);
        });
        builder.splineToSplineHeading(new Pose2d(0,40), Math.PI);
        builder.lineTo(new Vector2D(10,10));
        drive.followTrajectory(builder.build());
    }
}
