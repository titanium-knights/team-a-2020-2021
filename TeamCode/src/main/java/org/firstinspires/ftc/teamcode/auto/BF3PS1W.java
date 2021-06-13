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

@Autonomous(name = "BF3PS1W")
public class BF3PS1W extends LinearOpMode {
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

        int box = 2;
        UGContourRingPipeline.Height height = pipeline.getHeight();
        if (height == UGContourRingPipeline.Height.FOUR) {
            box = 2;
        } else if (height == UGContourRingPipeline.Height.ONE) {
            box = 1;}

        Pose2d startPose = new Pose2d (-64, 18, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        Trajectory shootPS = drive.trajectoryBuilder(startPose, false)
                .lineToConstantHeading(new Vector2d(-36.0, 19.0))
                .splineToConstantHeading(new Vector2d(-2.0, 7), Math.toRadians(0.0))
                .addDisplacementMarker(0.5,() -> {
                    shooterMotor.setPower(0.8);
                    shooterMotor2.setPower(0.8); })
                .build();
        Trajectory deliverA = drive.trajectoryBuilder(shootPS.end(), false)
                .lineToSplineHeading(new Pose2d(12.0, 12.0, Math.toRadians(180.0)))
                .splineToSplineHeading(new Pose2d(26.0, 56.0, Math.toRadians(0.0)), Math.toRadians(-90.0))
                .build();
        Trajectory deliverB = drive.trajectoryBuilder(shootPS.end(), false)
                .splineToLinearHeading(new Pose2d(32.0, 12.0, Math.toRadians(-90.0)), Math.toRadians(-90.0))
                .build();
        Trajectory deliverC = drive.trajectoryBuilder(shootPS.end(), false)
                .lineToSplineHeading(new Pose2d(48, 36, Math.toRadians(-90.0)))
                .splineToSplineHeading(new Pose2d(53, 46.0, Math.toRadians(-90.0)), Math.toRadians(90.0))
                .build();
        Trajectory intakeBB = drive.trajectoryBuilder(deliverA.end(), false)
                .splineToSplineHeading(new Pose2d(51.0, 52.0, Math.toRadians(-90)), Math.toRadians(-90.0))
                .lineToSplineHeading(new Pose2d(51.0, 26.0, Math.toRadians(-90.0)))
                .build();
        Trajectory shootBB = drive.trajectoryBuilder(intakeBB.end(), false)
                .splineToSplineHeading(new Pose2d(-6.0, 12.0, Math.toRadians(195.0)), Math.toRadians(180.0))
                .build();
        Trajectory parkA = drive.trajectoryBuilder(shootBB.end(), false)
                .splineToSplineHeading(new Pose2d(10.0, 12.0, Math.toRadians(180.0)), Math.toRadians(180.0))
                .build();
        Trajectory parkB = drive.trajectoryBuilder(deliverB.end(), false)
                .lineToSplineHeading(new Pose2d(12.0, 12.0, Math.toRadians(180.0)))
                .splineToSplineHeading(new Pose2d(12.0, 8.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                .build();
        Trajectory parkC = drive.trajectoryBuilder(deliverC.end(), false)
                .splineToSplineHeading(new Pose2d(53.0, 8.0, Math.toRadians(90)), Math.toRadians(0.0))
                .splineToSplineHeading(new Pose2d(12.0, 8.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                .build();

        waitForStart();
        wobbleGoal.lowerArm();
        /*
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.followTrajectory(shootPS);
        flap.setPosition(0.032); //High goal: 0.00      PS: 0.05
        sleep(1000);
        shooterServo.setPosition(12);
        sleep(400);
        shooterServo.setPosition(0);
        drive.turn(Math.toRadians(9));
        flap.setPosition(0.037);
        sleep(600);
        shooterServo.setPosition(12);
        sleep(400);
        shooterServo.setPosition(0);
        drive.turn(Math.toRadians(-15));
        flap.setPosition(0.032); //High goal: 0.00      PS: 0.05
        sleep(400);
        shooterServo.setPosition(12);
        sleep(400);
        shooterServo.setPosition(0);
        drive.turn(Math.toRadians(6));
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
                flap.setPosition(0.00);
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
                drive.followTrajectory(parkB);
                break;
            case 2:
                drive.followTrajectory(deliverC);
                wobbleGoal.release();
                drive.followTrajectory(parkC);
                break;
        }

         */

    }
}