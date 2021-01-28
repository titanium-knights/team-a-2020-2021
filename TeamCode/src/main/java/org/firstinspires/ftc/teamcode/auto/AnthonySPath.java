package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AnthonySPath extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        /*
        Trajectory trajectory = new TrajectoryBuilder("I did not see how this was supposed to start whoops")
                .splineTo(new Vector2d(-25, -60), Math.toRadians(0))
                .splineTo(new Vector2d(-5, -37.5), Math.toRadians(0))
                .splineTo(new Vector2d(37.5, -37.5), Math.toRadians(0))
                .splineTo(new Vector2d(-25, -12.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -12.5), Math.toRadians(180))
                .splineTo(new Vector2d(-25, -12.5), Math.toRadians(0))
                .splineTo(new Vector2d(37.5, -37.5), Math.toRadians(0))
                .build();
        */
    }
}
