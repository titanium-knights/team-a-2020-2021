package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.WobbleGoal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.io.IOException;

@Autonomous(name = "Path Test Op Moded")
public class PathTestOpMode extends LinearOpMode {
    boolean isBlue = false;

    private static final int CAMERA_WIDTH = 640; // width of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    public static int HORIZON = 0; // horizon value to tune

    private static final boolean DEBUG = true; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        WobbleGoal wobbleGoal = WobbleGoal.standard(hardwareMap);

        DcMotor shooterMotor = hardwareMap.dcMotor.get("shooter");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo shooterServo = hardwareMap.servo.get("pinball");
        Servo shooterFlap = hardwareMap.servo.get("Flap");

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

        waitForStart();

        int box = 0; // 0 is for closest box, 1 is for the middle box, 2 is for the farthest box
        UGContourRingPipeline.Height height = pipeline.getHeight();
        if (height == UGContourRingPipeline.Height.FOUR) {
            box = 2;
        } else if (height == UGContourRingPipeline.Height.ONE) {
            box = 1;
        }

        Trajectory shootTrajectory = drive.trajectoryBuilder(new Pose2d(-63, -40, Math.toRadians(180)), Math.toRadians(-75))
                .splineToLinearHeading(new Pose2d(-24, -61, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(12, -58, Math.toRadians(180)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-10, -40, Math.toRadians(180)), Math.toRadians(180))
                .build();

        drive.setPoseEstimate(shootTrajectory.start());
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setPower(-1);
        drive.followTrajectory(shootTrajectory);
        for (int i = 0; i < 3; ++i) {
            sleep(500);
            shooterServo.setPosition(0.15);
            sleep(500);
            shooterServo.setPosition(0);
            sleep(1000);
        }
        shooterMotor.setPower(0);

        TrajectoryBuilder builder = drive.trajectoryBuilder(shootTrajectory.end(), 0);
        switch (box) {
            case 0:
                builder.splineToLinearHeading(new Pose2d(12, -52, Math.toRadians(90)), Math.toRadians(-90));
                break;
            case 1:
                builder.splineToLinearHeading(new Pose2d(36, -28, Math.toRadians(90)), Math.toRadians(-90));
                break;
            case 2:
                builder.splineToLinearHeading(new Pose2d(60, -52, Math.toRadians(90)), Math.toRadians(-90));
                break;
        }
        Trajectory traj = builder.build();
        drive.followTrajectory(traj);
        wobbleGoal.lowerArm();
        sleep(1000);
        wobbleGoal.release();

        drive.followTrajectory(drive.trajectoryBuilder(traj.end(), Math.toRadians(90)).splineToLinearHeading(new Pose2d(12, 0, Math.toRadians(-90)), Math.toRadians(90)).build());
    }
}
