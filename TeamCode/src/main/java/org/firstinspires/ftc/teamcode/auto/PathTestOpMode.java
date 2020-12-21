package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.WobbleGoal;

import java.io.IOException;

@Autonomous(name = "Path Test Op Moded")
public class PathTestOpMode extends LinearOpMode {
    private Trajectory loadTrajectory(String name, TrajectoryGroupConfig groupConfig) {
        String filename = "trajectory/" + name + ".yaml";
        try {
            TrajectoryConfig config = TrajectoryConfigManager.loadConfig(hardwareMap.appContext.getAssets().open(filename));
            return config.toTrajectory(groupConfig);
        } catch (IOException e) {
            throw new RuntimeException("Unable to load trajectory: " + e.getMessage());
        }
    }

    private TrajectoryGroupConfig loadGroupConfig() {
        try {
            return TrajectoryConfigManager.loadGroupConfig(hardwareMap.appContext.getAssets().open("trajectory/_group.yaml"));
        } catch (IOException e) {
            throw new RuntimeException("Unable to load trajectory: " + e.getMessage());
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DistanceSensor bottomSensor = hardwareMap.get(DistanceSensor.class, "bottom_sensor");
        DistanceSensor topSensor = hardwareMap.get(DistanceSensor.class, "top_sensor");
        int wobbleGoalTarget = 0; // 0 = A, 1 = B, 2 = C

        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        CRServo shooterServo = hardwareMap.crservo.get("pinball");

        WobbleGoal wobbleGoal = WobbleGoal.standard(hardwareMap);
        Servo claw = hardwareMap.servo.get("wobble");

        TrajectoryGroupConfig groupConfig = loadGroupConfig();
        Trajectory preScan = loadTrajectory("redprescan", groupConfig);
        Trajectory shoot = loadTrajectory("redshoot", groupConfig);
        Trajectory[] trajectories = {
                loadTrajectory("reda", groupConfig),
                loadTrajectory("redb", groupConfig),
                loadTrajectory("redc", groupConfig)
        };
        Trajectory[] trajectories2 = {
                loadTrajectory("red2a", groupConfig),
                loadTrajectory("red2b", groupConfig),
                loadTrajectory("red2c", groupConfig)
        };

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(preScan.start());

        shooter.setPower(-1);

        drive.followTrajectory(preScan);

        double bottom = bottomSensor.getDistance(DistanceUnit.INCH);
        double top = topSensor.getDistance(DistanceUnit.INCH);
        if ((6.5 >= top && top >= 1) && (6 >= bottom && bottom >= 1)) {
            wobbleGoalTarget = 2;
        }
        else if ((8 >= bottom && bottom >= 4.5) && (top > 5)) {
            wobbleGoalTarget = 1;
        }

        telemetry.addData("b", bottom);
        telemetry.addData("t", top);
        telemetry.addData("Pos", wobbleGoalTarget);
        telemetry.update();

        if (wobbleGoalTarget == 0) {
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate(), 0).splineToLinearHeading(new Pose2d(-7, -32, Math.PI), 0).build());
        } else {
            drive.followTrajectory(shoot);
        }

        // Shooty shoot
        for (int i = 0; i < 3; ++i) {
            // Thwack thwack thwack
            // sleep(200);
            shooterServo.setPower(1);
            sleep(220);
            shooterServo.setPower(-1);
            sleep(200);
            shooterServo.setPower(0);
            sleep(220);
        }

        sleep(200);

        shooter.setPower(0);

        drive.followTrajectory(trajectories[wobbleGoalTarget]);

        wobbleGoal.lowerArm();
        claw.setPosition(1);
        sleep(1200);
        wobbleGoal.stopArm();
        wobbleGoal.liftArm();
        sleep(500);

        // if (wobbleGoalTarget == 1) {
            TrajectoryBuilder adjustmentBuilder = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.PI);
            adjustmentBuilder.splineToLinearHeading(new Pose2d(-36, -8, Math.PI), Math.PI);
            adjustmentBuilder.addTemporalMarker(0.5, wobbleGoal::lowerArm);
            adjustmentBuilder.addTemporalMarker(2.5, wobbleGoal::stopArm);
            drive.followTrajectory(adjustmentBuilder.build());

            TrajectoryBuilder nextBuilder = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.PI);
            // nextBuilder.splineToLinearHeading(new Pose2d(12, -11, Math.PI), Math.PI);
            nextBuilder.splineToLinearHeading(new Pose2d(-42, -13, Math.PI), Math.PI);
            drive.followTrajectory(nextBuilder.build());
        /* } else {
            TrajectoryBuilder nextBuilder = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.PI);
            // nextBuilder.splineToLinearHeading(new Pose2d(12, -11, Math.PI), Math.PI);
            nextBuilder.splineToLinearHeading(new Pose2d(-41, -12.5, Math.PI), Math.PI);
            nextBuilder.addTemporalMarker(0.5, wobbleGoal::lowerArm);
            nextBuilder.addTemporalMarker(2.5, wobbleGoal::stopArm);
            drive.followTrajectory(nextBuilder.build());
        } */

        claw.setPosition(0);
        sleep(1000);
        wobbleGoal.liftArm();
        sleep(1500);
        wobbleGoal.stopArm();

        drive.followTrajectory(trajectories2[wobbleGoalTarget]);
        wobbleGoal.lowerArm();
        claw.setPosition(1);
        sleep(800);
        wobbleGoal.stopArm();
        wobbleGoal.liftArm();
        sleep(1000);
        wobbleGoal.stopArm();

        if (wobbleGoalTarget == 0) {
            TrajectoryBuilder parkBuilder = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.PI);
            parkBuilder.splineToLinearHeading(new Pose2d(12, -12, Math.PI), Math.PI);
            drive.followTrajectory(parkBuilder.build());
        } else if (wobbleGoalTarget == 1) {
            TrajectoryBuilder parkBuilder = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.PI / 2);
            parkBuilder.splineToLinearHeading(new Pose2d(12, -12, Math.PI), Math.PI);
            drive.followTrajectory(parkBuilder.build());
        }
    }
}
