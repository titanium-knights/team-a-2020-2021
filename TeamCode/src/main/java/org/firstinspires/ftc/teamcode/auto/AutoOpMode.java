package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.*;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;

import java.util.Arrays;

@Autonomous(name = "Auto Op Mode")
public class AutoOpMode extends LinearOpMode {
    boolean isBlue = false;

    private static final int CAMERA_WIDTH = 640; // width of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    public static int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = true; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    private DcMotor shooterMotor;
    private DcMotor shooterMotor2;
    private Servo shooterServo;

    private WobbleGoal wobbleGoal;

    private void shoot() {
        shooterServo.setPosition(0);
        sleep(400);
        for (int i = 0; i < 3; ++i) {
            shooterServo.setPosition(0.15);
            sleep(400);
            shooterServo.setPosition(0);
            sleep(400);
        }
        shooterMotor.setPower(0);
        shooterMotor2.setPower(0);
    }

    private void grabGoal() {
        wobbleGoal.grab();
        sleep(500);
    }

    private void releaseGoal() {
        wobbleGoal.release();
    }

    private void raiseToFoldedPos() {
        wobbleGoal.liftArm();
        while (wobbleGoal.getPosition() < -2521) {
            sleep(10);
        }
        wobbleGoal.stopArm();
    }

    private void lowerToLoweredPos() {
        wobbleGoal.lowerArm();
        while (wobbleGoal.getPosition() > -5500) {
            sleep(10);
        }
        wobbleGoal.stopArm();
    }

    private void lowerToRaisedPos() {
        wobbleGoal.lowerArm();
        while (wobbleGoal.getPosition() > -2521) {
            sleep(10);
        }
        wobbleGoal.stopArm();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        wobbleGoal = WobbleGoal.standard(hardwareMap);

        shooterMotor = hardwareMap.dcMotor.get("shooter1");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2 = hardwareMap.dcMotor.get("shooter2");
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterServo = hardwareMap.servo.get("pinball");
        Servo shooterFlap = hardwareMap.servo.get("flap");

        int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME),
                    cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK,
                    cameraMonitorViewId);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(20);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(
                () -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        wobbleGoal.grab();
        shooterFlap.setPosition(0.025);

        waitForStart();

        int box = 0; // 0 is for closest box, 1 is for the middle box, 2 is for the farthest box

        UGContourRingPipeline.Height height = pipeline.getHeight();
        if (height == UGContourRingPipeline.Height.FOUR) {
            box = 2;
        } else if (height == UGContourRingPipeline.Height.ONE) {
            box = 1;
        }

        telemetry.addData("Chosen Path", box);
        telemetry.update();

        Trajectory trajectory = null;

        shooterMotor.setPower(1);
        shooterMotor2.setPower(1);

        if (isBlue) {
            // Blue stuff
            trajectory = drive.trajectoryBuilder(new Pose2d(-60, 25, Math.toRadians(180)))
                    .splineToLinearHeading(new Pose2d(10, 25, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.setPoseEstimate(trajectory.start());
                    drive.followTrajectory(trajectory);
                    shoot();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(33, 50, Math.toRadians(180)), Math.toRadians(0))
                    .addSpatialMarker(new Vector2d(33, 50), () -> {
                        wobbleGoal.lowerArm();
                    })
                    .splineToLinearHeading(new Pose2d(50, 60, Math.toRadians(180)), Math.toRadians(0))
                    .addSpatialMarker(new Vector2d(50, 60), () -> {
                        wobbleGoal.release();
                    })
                    .splineToLinearHeading(new Pose2d(-49, 50, Math.toRadians(180)), Math.toRadians(0)).build();
        } else {
            switch (box) {
                case 0:
                    trajectory = drive.trajectoryBuilder(new Pose2d(-63, -50, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-2, -40, Math.toRadians(180)), Math.toRadians(0))
                            .build();
                    drive.setPoseEstimate(trajectory.start());
                    drive.followTrajectory(trajectory);
                    sleep(1000);
                    shoot();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-3, -62, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    lowerToLoweredPos();
                    releaseGoal();
                    sleep(500);
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(-33, -29, Math.toRadians(0)), Math.toRadians(180)).build();
                    drive.followTrajectory(trajectory);
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(180))
                            .lineTo(new Vector2d(-41, -29), new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                            ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).build();
                    drive.followTrajectory(trajectory);
                    grabGoal();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(14, -47, Math.toRadians(90)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    releaseGoal();
                    break;
                case 1:
                    // middle box
                    trajectory = drive.trajectoryBuilder(new Pose2d(-63, -50, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-25, -60, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-2, -40, Math.toRadians(180)), Math.toRadians(0))
                            .build();
                    drive.setPoseEstimate(trajectory.start());
                    drive.followTrajectory(trajectory);
                    shoot();
                    intake.togglePower();
                    trajectory = drive.trajectoryBuilder(trajectory.end())
                            .forward(10)
                            .build();
                            drive.followTrajectory(trajectory);
                            intake.togglePower();
                    trajectory = drive.trajectoryBuilder(trajectory.end())
                            .forward(-10)
                            .build();
                            drive.followTrajectory(trajectory);
                        shoot();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(23, -40, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    lowerToLoweredPos();
                    releaseGoal();
                    sleep(500);
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(-33, -26, Math.toRadians(0)), Math.toRadians(180)).build();
                    drive.followTrajectory(trajectory);
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(180))
                            .lineTo(new Vector2d(-41, -26), new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                            ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).build();
                    drive.followTrajectory(trajectory);
                    grabGoal();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(13.5, -35, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    releaseGoal();
                    sleep(500);
                    break;
                case 2:
                    trajectory = drive.trajectoryBuilder(new Pose2d(-63, -50, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-25, -60, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-2, -40, Math.toRadians(180)), Math.toRadians(0))
                            .build();
                    drive.setPoseEstimate(trajectory.start());
                    drive.followTrajectory(trajectory);
                    shoot();
                    intake.togglePower();
                    trajectory = drive.trajectoryBuilder(trajectory)
                        .forward(10)
                        .build();
                        drive.followTrajectory(trajectory);
                        intake.togglePower();
                trajectory = drive.trajectoryBuilder(trajectory)
                        .forward(-10)
                        .build();
                        drive.followTrajectory(trajectory);
                shoot();                        
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(42, -62, Math.toRadians(180)), Math.toRadians(0))
                            .build();
                    drive.followTrajectory(trajectory);
                    lowerToLoweredPos();
                    releaseGoal();
                    sleep(500);
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(-20, -12.5, Math.toRadians(0)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-31, -26, Math.toRadians(0)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(180))
                            .lineTo(new Vector2d(-43, -26), new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                            ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).build();
                    drive.followTrajectory(trajectory);
                    grabGoal();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-20, -12.5, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(42, -62, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    releaseGoal();
                    sleep(500);
                    break;
            }
        }

        trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(box == 0 ? 90 : 180))
                .splineToLinearHeading(new Pose2d(8, 0), Math.toRadians(180)).build();
        drive.followTrajectory(trajectory);
    }
}
