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

@Autonomous(name = "RF3PS1WStack")
public class RF3PS1WStack extends LinearOpMode {
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
        shooterServo.setPosition(0.00);

        int box = 0;
        UGContourRingPipeline.Height height = pipeline.getHeight();
        if (height == UGContourRingPipeline.Height.FOUR) {
            box = 2;
        } else if (height == UGContourRingPipeline.Height.ONE) {
            box = 1;}

        Pose2d startPose = new Pose2d (-64, -18, Math.toRadians(181));
        drive.setPoseEstimate(startPose);
        Trajectory shootPS = drive.trajectoryBuilder(startPose, false)
                .lineToConstantHeading(new Vector2d(-36.0, -19.0))
                .splineToConstantHeading(new Vector2d(-2.0, -19), Math.toRadians(0.0))
                .addDisplacementMarker(0.5,() -> {
                    shooterMotor.setPower(0.8);
                    shooterMotor2.setPower(0.8); })
                .build();
        Trajectory deliverA = drive.trajectoryBuilder(shootPS.end(), false)
                .lineToSplineHeading(new Pose2d(12.0, -12.0, Math.toRadians(180.0)))
                .splineToSplineHeading(new Pose2d(26.0, -57.0, Math.toRadians(0.0)), Math.toRadians(-90.0))
                .build();
        Trajectory deliverB = drive.trajectoryBuilder(shootPS.end(), false)
                .splineToLinearHeading(new Pose2d(36.0, -21.0, Math.toRadians(90.0)), Math.toRadians(-90.0))
                .build();
        Trajectory deliverC = drive.trajectoryBuilder(shootPS.end(), false)
                .lineToSplineHeading(new Pose2d(48, -36, Math.toRadians(90.0)))
                .splineToSplineHeading(new Pose2d(53, -48.0, Math.toRadians(90.0)), Math.toRadians(-90.0))
                .build();
        Trajectory intakeBB = drive.trajectoryBuilder(deliverA.end(), false)
                .splineToSplineHeading(new Pose2d(51.0, -52.0, Math.toRadians(90.0)), Math.toRadians(90.0))
                .lineToSplineHeading(new Pose2d(51.0, -26.0, Math.toRadians(90.0)))
                .build();
        Trajectory shootBB = drive.trajectoryBuilder(intakeBB.end(), false)
                .splineToSplineHeading(new Pose2d(-8.0, -35.0, Math.toRadians(178.0)), Math.toRadians(0.0))
                .build();
        Trajectory intakeB = drive.trajectoryBuilder(deliverB.end(), false)
                .splineToSplineHeading(new Pose2d(36, -16.0, Math.toRadians(90.0)), Math.toRadians(90.0))
                .splineToSplineHeading(new Pose2d(12.0, -36.0, Math.toRadians(180.0)), Math.toRadians(180.0))
                .splineToSplineHeading(new Pose2d(-24.0, -36.0, Math.toRadians(180.0)), Math.toRadians(180.0))
                .build();
        Trajectory intakeC = drive.trajectoryBuilder(deliverC.end(), false)
                .splineToSplineHeading(new Pose2d(30.0, -40.0, Math.toRadians(180.0)), Math.toRadians(90.0))
                .splineToSplineHeading(new Pose2d(6.0, -36.0, Math.toRadians(180.0)), Math.toRadians(180.0))
                .lineToSplineHeading(new Pose2d(-28.0, -36.0, Math.toRadians(180.0)), //6
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(0.3 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory shoot1 = drive.trajectoryBuilder(intakeB.end(), false)
                .splineToSplineHeading(new Pose2d(-8.0, -37.0, Math.toRadians(179.0)), Math.toRadians(0.0))
                .build();
        Trajectory intake2 = drive.trajectoryBuilder(shoot1.end(), false)
                .lineToSplineHeading(new Pose2d(-40.0, -36.0, Math.toRadians(180.0)), //6
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(0.3 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory shoot2 = drive.trajectoryBuilder(intake2.end(), false)
                .splineToSplineHeading(new Pose2d(-8.0, -37.0, Math.toRadians(179.0)), Math.toRadians(0.0))
                .build();
        Trajectory parkA = drive.trajectoryBuilder(shootBB.end(), false)
                .splineToSplineHeading(new Pose2d(10.0, -12.0, Math.toRadians(180.0)), Math.toRadians(180.0))
                .build();
        Trajectory park = drive.trajectoryBuilder(shoot1.end(), false)
                .splineToSplineHeading(new Pose2d(12.0, -12.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                .build();

        waitForStart();
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.followTrajectory(shootPS);
        flap.setPosition(0.032); //High goal: 0.00      PS: 0.05
        sleep(1000);
        shooterServo.setPosition(12);
        sleep(400);
        shooterServo.setPosition(0);
        drive.turn(Math.toRadians(8));
        flap.setPosition(0.036);
        sleep(600);
        shooterServo.setPosition(12);
        sleep(400);
        shooterServo.setPosition(0);
        drive.turn(Math.toRadians(-13.5));
        flap.setPosition(0.031); //High goal: 0.00      PS: 0.05
        sleep(400);
        shooterServo.setPosition(12);
        sleep(400);
        shooterServo.setPosition(0);
        drive.turn(Math.toRadians(5.5));
        wobbleGoal.lowerArm();
        shooterMotor.setPower(0);
        shooterMotor2.setPower(0);

        switch (box) {
            case 0:
                drive.followTrajectory(deliverA);
                wobbleGoal.release();
                intake.togglePower();
                drive.followTrajectory(intakeBB);
                shooterMotor.setPower(1);
                shooterMotor2.setPower(1);
                wobbleGoal.liftArm();
                wobbleGoal.grab();
                drive.followTrajectory(shootBB);
                flap.setPosition(0.013);
                intake.togglePower();
                sleep(1000);
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.12);
                    sleep(400);
                    shooterServo.setPosition(0);
                    sleep(200);
                }
                sleep(200);
                wobbleGoal.lowerArm();
                wobbleGoal.release();
                shooterMotor.setPower(0);
                shooterMotor2.setPower(0);
                drive.followTrajectory(parkA);
                break;
            case 1:
                drive.followTrajectory(deliverB);
                wobbleGoal.release();
                intake.togglePower();
                drive.followTrajectory(intakeB);
                wobbleGoal.liftArm();
                wobbleGoal.grab();
                shooterMotor.setPower(1.0);
                shooterMotor2.setPower(1.0);
                flap.setPosition(0.013);
                drive.followTrajectory(shoot1);
                intake.togglePower();
                sleep(1600);
                for (int i = 0; i < 2; ++i) {
                    shooterServo.setPosition(0.12);
                    sleep(300);
                    shooterServo.setPosition(0);
                    sleep(250);
                }
                wobbleGoal.lowerArm();
                wobbleGoal.release();
                drive.followTrajectory(park);
                break;
            case 2:
                drive.followTrajectory(deliverC);
                wobbleGoal.release();
                intake.togglePower();
                drive.followTrajectory(intakeC);
                wobbleGoal.liftArm();
                wobbleGoal.grab();
                shooterMotor.setPower(0.9);
                shooterMotor2.setPower(0.9);
                drive.followTrajectory(shoot1);
                flap.setPosition(0.013);
                intake.togglePower();
                sleep(1000);
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.12);
                    sleep(400);
                    shooterServo.setPosition(0);
                    sleep(250);
                }
                shooterMotor.setPower(0);
                shooterMotor2.setPower(0);
                intake.togglePower();
                drive.followTrajectory(intake2);
                shooterMotor.setPower(0.9);
                shooterMotor2.setPower(0.9);
                drive.followTrajectory(shoot2);
                intake.togglePower();
                sleep(1000);
                for (int i = 0; i < 3; ++i) {
                shooterServo.setPosition(0.12);
                sleep(400);
                shooterServo.setPosition(0);
                sleep(200);
                }
                wobbleGoal.lowerArm();
                wobbleGoal.release();
                drive.followTrajectory(park);
                break;
        }
    }
}