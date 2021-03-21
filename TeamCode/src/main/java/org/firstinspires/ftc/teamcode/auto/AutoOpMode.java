package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.*;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;

@Autonomous(name = "Auto Op Mode")
public class AutoOpMode extends LinearOpMode {
    boolean isBlue = false;

    private static final int CAMERA_WIDTH = 640; // width of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    public static int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = true; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = false; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    private DcMotor shooterMotor;
    private DcMotor shooterMotor2;
    private Servo shooterServo;

    private WobbleGoal wobbleGoal;

    private void shoot() {
        for (int i = 0; i < 3; ++i) {
            shooterServo.setPosition(0.15);
            sleep(300);
            shooterServo.setPosition(0);
            sleep(300);
        }
        shooterMotor.setPower(0);
        shooterMotor2.setPower(0);
    }

    private void grabGoal() {
        wobbleGoal.lowerArm();
        sleep(1000);
        wobbleGoal.release();
    }

    private void releaseGoal() {
        wobbleGoal.grab();
        sleep(1000);
        wobbleGoal.liftArm();
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
        wobbleGoal.liftArm();

        waitForStart();

        int box = 0; // 0 is for closest box, 1 is for the middle box, 2 is for the farthest box

        UGContourRingPipeline.Height height = pipeline.getHeight();
        if (height == UGContourRingPipeline.Height.FOUR) {
            box = 2;
        } else if (height == UGContourRingPipeline.Height.ONE) {
            box = 1;
        }

        Trajectory trajectory = null;

        shooterMotor.setPower(-1);
        shooterMotor2.setPower(-1);

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
                            .splineToLinearHeading(new Pose2d(-10, -40, Math.toRadians(180)), Math.toRadians(0))
                            .build();
                    drive.setPoseEstimate(trajectory.start());
                    drive.followTrajectory(trajectory);
                    // shoot();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(15, -38, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    // releaseGoal();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(15, -13, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-48, -13, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    // grabGoal();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(15, -13, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(15, -38, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    // releaseGoal();
                    break;
                case 1:
                    // middle box
                    trajectory = drive.trajectoryBuilder(new Pose2d(-63, -50, Math.toRadians(180)))
                            .splineToLinearHeading(new Pose2d(-25, -60, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-5, -37.5, Math.toRadians(180)), Math.toRadians(0))
                            .build();
                    drive.setPoseEstimate(trajectory.start());
                    drive.followTrajectory(trajectory);
                    // shoot();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(37.5, -37.5, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    // releaseGoal();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-25, -12.5, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-50, -12.5, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    // grabGoal();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-25, -12.5, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(37.5, -37.5, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    // releaseGoal();
                    break;
                case 2:
                    trajectory = drive.trajectoryBuilder(new Pose2d(-63, -50, Math.toRadians(180)))
                            .splineToLinearHeading(new Pose2d(-24, -26, Math.toRadians(180)), Math.toRadians(0))
                            .build();
                    drive.setPoseEstimate(trajectory.start());
                    drive.followTrajectory(trajectory);
                    shoot();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(24, -37, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(54, -37, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(60, -53, Math.toRadians(180)), Math.toRadians(0))
                            .build();
                    drive.followTrajectory(trajectory);
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-25, -12.5, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-50, -12.5, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    // grabGoal();
                    trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-25, -12.5, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(60, -53, Math.toRadians(180)), Math.toRadians(0)).build();
                    drive.followTrajectory(trajectory);
                    // releaseGoal();
                    break;
            }
        }

        trajectory = drive.trajectoryBuilder(trajectory.end(), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(0, 0), Math.toRadians(180)).build();
        drive.followTrajectory(trajectory);
    }
}