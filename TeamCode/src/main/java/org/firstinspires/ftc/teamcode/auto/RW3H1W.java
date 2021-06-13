package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.WobbleGoal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Arrays;

@Autonomous(name = "RW3H1W")
public class RW3H1W extends LinearOpMode {

    private static final int CAMERA_WIDTH = 640; // width of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    public static int HORIZON = 100; // horizon value to tune
    private static final boolean DEBUG = true; // if debug is wanted, change to true
    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam
    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        WobbleGoal wobbleGoal = WobbleGoal.standard(hardwareMap);
        Intake intake = Intake.standard(hardwareMap);
        DcMotor shooterMotor = hardwareMap.dcMotor.get("shooter1");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor shooterMotor2 = hardwareMap.dcMotor.get("shooter2");
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo shooterServo = hardwareMap.servo.get("pinball");
        Servo flap = hardwareMap.servo.get("Flap");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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

        wobbleGoal.liftArm();
        wobbleGoal.grab();
        flap.setPosition(0.00); //High goal: 0.00      PS: 0.05
        shooterServo.setPosition(0.00);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int box = 0;
        UGContourRingPipeline.Height height = pipeline.getHeight();
        if (height == UGContourRingPipeline.Height.FOUR) {
            box = 2;
        } else if (height == UGContourRingPipeline.Height.ONE) {
            box = 1;}

        Pose2d startPose = new Pose2d (-64, -56, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        //A
        Trajectory deliverA = drive.trajectoryBuilder(startPose, false)
                .lineToConstantHeading(new Vector2d(-4.0, -64))
                .build();
        Trajectory shootA = drive.trajectoryBuilder(deliverA.end(), false)
                .splineToLinearHeading(new Pose2d(-6, -58.0, Math.toRadians(196.0)), Math.toRadians(180.0))
                .addDisplacementMarker(8, () -> {
                    shooterMotor.setPower(0.9);
                    shooterMotor2.setPower(0.9);
                    wobbleGoal.liftArm();})
                .build();
        Trajectory parkA = drive.trajectoryBuilder((shootA.end()), false)
                .splineToSplineHeading(new Pose2d(-18.0, -56.0, Math.toRadians(135.0)), Math.toRadians(180.0))
                .addDisplacementMarker(8, () -> {
                    wobbleGoal.lowerArm();
                    wobbleGoal.release();
                })
                .splineToSplineHeading(new Pose2d(6.0, -42.0, Math.toRadians(90.0)), Math.toRadians(0.0))
                .build();
        //B
        Trajectory deliverB = drive.trajectoryBuilder((startPose), false)
                .lineToConstantHeading(new Vector2d(-4.0, -56.0))
                .splineToSplineHeading(new Pose2d(24.0, -52.0, Math.toRadians(225.0)), Math.toRadians(0.0))
                .build();
        Trajectory shootB = drive.trajectoryBuilder((deliverB.end()), false)
                .splineToLinearHeading(new Pose2d(-6, -58.0, Math.toRadians(188.0)), Math.toRadians(180.0))
                .addDisplacementMarker( 8, () -> {
                    shooterMotor.setPower(0.9);
                    shooterMotor2.setPower(0.9);
                    wobbleGoal.liftArm();
                    wobbleGoal.grab(); })
                .build();
        //C
        Trajectory deliverC = drive.trajectoryBuilder(startPose, false)
                .lineToConstantHeading(new Vector2d(46.0, -66))
                .build();
        Trajectory shootC = drive.trajectoryBuilder(deliverC.end(), false)
                .splineToLinearHeading(new Pose2d(-6, -58.0, Math.toRadians(188.0)), Math.toRadians(180.0))
                .addDisplacementMarker( 9, () -> {
                    shooterMotor.setPower(0.9);
                    shooterMotor2.setPower(0.9);
                    wobbleGoal.liftArm();
                    wobbleGoal.grab(); })
                .build();
        Trajectory park = drive.trajectoryBuilder((shootC.end()), false)
                .splineToLinearHeading(new Pose2d(8.0, -66.0, Math.toRadians(180.0)), Math.toRadians(180.0))
                .addDisplacementMarker(8, () -> {
                    wobbleGoal.lowerArm();
                    wobbleGoal.release();
                })
                .build();


        waitForStart();
        flap.setPosition(0.013);
        switch (box) {
            case 0:
                wobbleGoal.lowerArm();
                sleep(100);
                drive.followTrajectory(deliverA);
                wobbleGoal.release();
                drive.followTrajectory(shootA);
                sleep(1200);
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.12);
                    sleep(400);
                    shooterServo.setPosition(0);
                    sleep(200);
                }
                sleep(200);
                shooterMotor.setPower(0);
                shooterMotor2.setPower(0);
                sleep(15000);
                drive.followTrajectory(parkA);
                break;
            case 1:
                wobbleGoal.lowerArm();
                sleep(100);
                drive.followTrajectory(deliverB);
                wobbleGoal.release();
                drive.followTrajectory(shootB);
                sleep(1000);
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.12);
                    sleep(400);
                    shooterServo.setPosition(0);
                    sleep(200);
                }
                drive.followTrajectory(park);
                break;
            case 2:
                wobbleGoal.lowerArm();
                sleep(100);
                drive.followTrajectory(deliverC);
                wobbleGoal.release();
                drive.followTrajectory(shootC);
                sleep(1000);
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.12);
                    sleep(400);
                    shooterServo.setPosition(0);
                    sleep(200);
                }
                drive.followTrajectory(park);
                break;
        }
        sleep(1000);
    }
}