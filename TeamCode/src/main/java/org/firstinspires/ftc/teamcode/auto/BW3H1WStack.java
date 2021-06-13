package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

@Autonomous(name = "BW3H1WStack")
public class BW3H1WStack extends LinearOpMode {

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

        int box = 0;
        UGContourRingPipeline.Height height = pipeline.getHeight();
        if (height == UGContourRingPipeline.Height.FOUR) {
            box = 2;
        } else if (height == UGContourRingPipeline.Height.ONE) {
            box = 1;}

        Pose2d startPose = new Pose2d (-64, 56, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        //A
        Trajectory deliverA1 = drive.trajectoryBuilder(startPose, false)
                .lineToConstantHeading(new Vector2d(-2.0, 62.5))
                    .addDisplacementMarker(60,() -> {
                        wobbleGoal.release(); })
                .build();
        Trajectory shootA = drive.trajectoryBuilder(deliverA1.end(), false)
                .splineToConstantHeading(new Vector2d(-8.0, 64), Math.toRadians(180.0))
                .splineToConstantHeading(new Vector2d(-10.0, 34.0), Math.toRadians(180.0))
                .addDisplacementMarker( 8, () -> {
                    wobbleGoal.liftArm();
                    wobbleGoal.grab();
                    shooterMotor.setPower(0.9);
                    shooterMotor2.setPower(0.9); })
                .build();
        Trajectory parkA = drive.trajectoryBuilder((shootA.end()), true)
                .splineToConstantHeading(new Vector2d(-18.0, 60.5), Math.toRadians(180.0))
                .splineToConstantHeading(new Vector2d(-18.0, 40.5), Math.toRadians(180.0))
                .splineToConstantHeading(new Vector2d(6.0, 36.0), Math.toRadians(0.0))
                .build();

        //B
        Trajectory deliverB1 = drive.trajectoryBuilder((startPose), false)
                .lineToConstantHeading(new Vector2d(-23.0, 56.0))
                .splineToConstantHeading(new Vector2d(13.0, 38.0), Math.toRadians(180.0))
                    .addDisplacementMarker(80, () -> {
                        wobbleGoal.release(); })
                .build();
        Trajectory shootB = drive.trajectoryBuilder((deliverB1.end()), false)
                .splineToConstantHeading(new Vector2d(-10, 34.0), Math.toRadians(180.0))
                    .addDisplacementMarker( 8, () -> {
                        wobbleGoal.liftArm();
                        wobbleGoal.grab();
                        shooterMotor.setPower(0.9);
                        shooterMotor2.setPower(0.9); })
                .build();
        Trajectory intakeB = drive.trajectoryBuilder((shootB.end()), false)
                .splineToConstantHeading(new Vector2d(-24.0, 36.0), Math.toRadians(0.0))
                    .addDisplacementMarker(1, () -> {
                        shooterMotor.setPower(0);
                        shooterMotor2.setPower(0);
                        intake.togglePower();})
                .splineToConstantHeading(new Vector2d(-10, 34.0), Math.toRadians(0.0), //6
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(0.7 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(20, () -> {
                        shooterMotor.setPower(0.9);
                        shooterMotor2.setPower(0.9);})
                .build();
        Trajectory parkB = drive.trajectoryBuilder((shootB.end()), false)
                .splineToConstantHeading(new Vector2d(6.0, 41.0), Math.toRadians(180.0))
                .build();

        //C
        Trajectory deliverC1 = drive.trajectoryBuilder(startPose, false)
                .lineToConstantHeading(new Vector2d(44,58))
                    .addDisplacementMarker( () -> {
                        wobbleGoal.release();})
                .build();
        Trajectory shootC = drive.trajectoryBuilder(deliverC1.end(), false)
                .splineToLinearHeading(new Pose2d(-9.3, 34.0, Math.toRadians(179.0)), Math.toRadians(180.0))
                .addDisplacementMarker( 8, () -> {
                    wobbleGoal.liftArm();
                    wobbleGoal.grab();
                    shooterMotor.setPower(0.9);
                    shooterMotor2.setPower(0.9); })
                .build();
        Trajectory intakeC1 = drive.trajectoryBuilder((shootC.end()), false)
                .splineToConstantHeading(new Vector2d(-17.0, 36.0), Math.toRadians(180.0), //
                        new MinVelocityConstraint(Arrays.asList(
                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                            new MecanumVelocityConstraint(0.2 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-8, 34.0), Math.toRadians(180.0))
                .addDisplacementMarker(4, () -> {
                    shooterMotor.setPower(0);
                    shooterMotor2.setPower(0);
                    intake.togglePower();})
                .addDisplacementMarker(32, () -> {
                    shooterMotor.setPower(0.9);
                    shooterMotor2.setPower(0.9);})
                .build();
        Trajectory intakeC2 = drive.trajectoryBuilder((intakeC1.end()), false)
                .splineToConstantHeading(new Vector2d(-32.0, 36.0), Math.toRadians(180.0),
                        new MinVelocityConstraint(Arrays.asList(
                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                            new MecanumVelocityConstraint(0.3 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(1, () -> {
                        shooterMotor.setPower(0);
                        shooterMotor2.setPower(0);
                        intake.togglePower();})
                .splineToConstantHeading(new Vector2d(-7, 34.0), Math.toRadians(180.0))
                    .addDisplacementMarker(48, () -> {
                        shooterMotor.setPower(0.9);
                        shooterMotor2.setPower(0.9);})
                .build();
        Trajectory parkC = drive.trajectoryBuilder((intakeC2.end()), false)
                .splineToConstantHeading(new Vector2d(8.0, 56), Math.toRadians(180.0))
                .build();

        waitForStart();
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        switch (box) {
            case 0:
                flap.setPosition(0.013);
                wobbleGoal.lowerArm();
                sleep(100);
                drive.followTrajectory(deliverA1);
                drive.followTrajectory(shootA);
                sleep(1000);
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.12);
                    sleep(400);
                    shooterServo.setPosition(0);
                    sleep(200);
                }
                sleep(500);
                drive.followTrajectory(parkA);
                break;
            case 1:
                flap.setPosition(0.013);
                wobbleGoal.lowerArm();
                drive.followTrajectory(deliverB1);
                drive.followTrajectory(shootB);
                sleep(1000);
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.12);
                    sleep(300);
                    shooterServo.setPosition(0);
                    sleep(200);
                }
                sleep(200);
                drive.followTrajectory(intakeB);
                intake.togglePower();
                sleep(1000);
                for (int i = 0; i < 2; ++i) {
                    shooterServo.setPosition(0.12);
                    sleep(300);
                    shooterServo.setPosition(0);
                    sleep(200);
                }
                sleep(500);
                drive.followTrajectory(parkB);
                break;
            case 2:
                flap.setPosition(0.013);
                wobbleGoal.lowerArm();
                drive.followTrajectory(deliverC1);
                drive.followTrajectory(shootC);
                sleep(750);
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.12);
                    sleep(400);
                    shooterServo.setPosition(0);
                    sleep(200);
                }
                sleep(100);
                drive.followTrajectory(intakeC1);
                sleep(100);
                intake.togglePower();
                sleep(700);
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.12);
                    sleep(400);
                    shooterServo.setPosition(0);
                    sleep(200);
                }
                sleep(100);
                drive.followTrajectory(intakeC2);
                intake.togglePower();
                sleep(700);
                for (int i = 0; i < 2; ++i) {
                    shooterServo.setPosition(0.12);
                    sleep(400);
                    shooterServo.setPosition(0);
                    sleep(200);
                }
                sleep(500);
                drive.followTrajectory(parkC);
                break;
            }
            wobbleGoal.release();
            wobbleGoal.lowerArm();
            sleep(500);
        }
    }