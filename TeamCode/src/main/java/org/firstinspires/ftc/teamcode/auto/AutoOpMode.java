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

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(
                () -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        waitForStart();

        int box = 0; // 0 is for closest box, 1 is for the middle box, 2 is for the farthest box

        UGContourRingPipeline.Height height = pipeline.getHeight();
        if (height == UGContourRingPipeline.Height.FOUR) {
            box = 2;
        } else if (height == UGContourRingPipeline.Height.ONE) {
            box = 1;
        }

        Trajectory trajectory = null;

        if (isBlue) {
            // Blue stuff
            trajectory = drive.trajectoryBuilder(new Pose2d(-60, 25, Math.toRadians(0)))
                    .splineToLinearHeading(new Pose2d(10, 25, Math.toRadians(0)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(50, 60, Math.toRadians(0)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(-49, 50, Math.toRadians(0)), Math.toRadians(0)).build();
        } else {
            switch (box) {
                case 0:
                    trajectory = drive.trajectoryBuilder(new Pose2d(-63, -38, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(15, -38, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(15, -13, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-63, -13, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(15, -13, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(15, -38, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(10, 0, Math.toRadians(0)), Math.toRadians(0)).build();
                    break;
                case 1:
                    // middle box
                    trajectory = drive.trajectoryBuilder(new Pose2d(-50, -50, Math.toRadians(0)))
                            .splineToLinearHeading(new Pose2d(-25, -60), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-5, -37.5), Math.toRadians(0))
                            .addSpatialMarker(new Vector2d(-5, -37.5), () -> {
                                // shoot
                            }).splineToLinearHeading(new Pose2d(37.5, -37.5), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-25, -12.5), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-50, -12.5), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-25, -12.5), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(37.5, -37.5), Math.toRadians(0)).build();
                    break;
                /*
                 * trajectory = drive.trajectoryBuilder(new Pose2d(-60, -25, Math.toRadians(0)))
                 * .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(0)),
                 * Math.toRadians(0)) .addSpatialMarker(new Vector2d(0, -36), () -> { // shoot
                 * }).splineToLinearHeading(new Pose2d(35, -25, Math.toRadians(0)),
                 * Math.toRadians(0)) .splineToLinearHeading(new Pose2d(10, -12,
                 * Math.toRadians(0)), Math.toRadians(0)).build();
                 */
                case 2:
                    trajectory = drive.trajectoryBuilder(new Pose2d(-64, -26, Math.toRadians(0)))
                            .splineToLinearHeading(new Pose2d(-24, -26, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(24, -37, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(54, -37, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(60, -53, Math.toRadians(0)), Math.toRadians(0)).build();
                    break;
            }
        }

        drive.setPoseEstimate(trajectory.start());
        drive.followTrajectory(trajectory);
    }
}