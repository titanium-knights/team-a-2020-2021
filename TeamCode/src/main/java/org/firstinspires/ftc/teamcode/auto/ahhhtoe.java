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

@Autonomous(name = "ahhhtoe")
public class ahhhtoe extends LinearOpMode {
    boolean isBlue = false;

    //random vision stuff
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

        // init stuff
        wobbleGoal.liftArm();
        wobbleGoal.grab();
        flap.setPosition(0.03);
        shooterServo.setPosition(0.00);


            int box = 0; // 0 is for closest box, 1 is for the middle box, 2 is for the farthest box
            UGContourRingPipeline.Height height = pipeline.getHeight();
            if (height == UGContourRingPipeline.Height.FOUR) {
                box = 2;
            } else if (height == UGContourRingPipeline.Height.ONE) {
                box = 1;}

        waitForStart();



        //defines start pose
        Pose2d startPose = new Pose2d (-64, -42, Math.toRadians(180)); //64, 42
        drive.setPoseEstimate(startPose);

        //initial trajectory that drives to the launch line and shoots the pre-loaded rings
        Trajectory shootTrajectory = drive.trajectoryBuilder((startPose), true)
                .splineToConstantHeading(new Vector2d(-36, -60), Math.toRadians(0)) //-36, -52
                .lineToConstantHeading(new Vector2d(-18, -60)) //-18, -52
                .splineToConstantHeading(new Vector2d(-2, -43), Math.toRadians(0)) //1, -36.5
                .build();

        //Trajectories for A (0 stack)
        Trajectory deliverA1 = drive.trajectoryBuilder((shootTrajectory.end()), true)
                .splineToConstantHeading(new Vector2d(0.0, -60.5), Math.toRadians(0))
                .build();
        Trajectory backawayA1 = drive.trajectoryBuilder((deliverA1.end()), false)
                .forward(6)
                .build();
        Trajectory runWobbleA = drive.trajectoryBuilder((backawayA1.end()), false)
                .splineToLinearHeading(new Pose2d(-30.0, -26.0, Math.toRadians(0.0)), Math.toRadians(180.0)) //pick up wobble2
                .build();
        Trajectory wobbleSlowA = drive.trajectoryBuilder((runWobbleA.end()), false)
                .back(6, //6
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(0.3 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory deliverA2 = drive.trajectoryBuilder((wobbleSlowA.end()), false)
                .splineToLinearHeading(new Pose2d(-8.0, -60.5, Math.toRadians(180)), Math.toRadians(0))
                .build();
        Trajectory parkA = drive.trajectoryBuilder((deliverA2.end()), false)
                .lineToConstantHeading(new Vector2d(-12.0, -60.5))
                .splineToConstantHeading(new Vector2d(12.0, -24.0), Math.toRadians(0))
                .build();

        //Trajectories for B (1 stack)
        Trajectory intakeB = drive.trajectoryBuilder((shootTrajectory.end()), false)
                .lineToConstantHeading(new Vector2d(-24.0, -40),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(0.3 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory shootB = drive.trajectoryBuilder((intakeB.end()), false)
                .lineToConstantHeading(new Vector2d(-2.0, -43))
                .build();
        Trajectory deliverB1 = drive.trajectoryBuilder((shootB.end()), false)
                .back(24.0)
                .build();
        Trajectory runWobbleB = drive.trajectoryBuilder((deliverB1.end()), false)
                .splineToSplineHeading(new Pose2d(-30.0, -24.0, Math.toRadians(0.0)), Math.toRadians(180.0))
                .build();
        Trajectory wobbleSlowB = drive.trajectoryBuilder((runWobbleB.end()), false)
                .back(9, //6
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(0.3 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory deliverB2 = drive.trajectoryBuilder((wobbleSlowB.end()), false)
                .splineToLinearHeading(new Pose2d(15.0, -36.5, Math.toRadians(180.0)), Math.toRadians(0.0))
                .build();
        Trajectory parkB = drive.trajectoryBuilder((deliverB2.end()), false)
                .forward(12.0)
                .build();

        //Trajectories for C (4 stack)
        Trajectory strafeC = drive.trajectoryBuilder((shootTrajectory.end()), false)
                .strafeRight(1)
                .build();
        Trajectory intakeC = drive.trajectoryBuilder((strafeC.end()), false)
                .forward(20, //
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(0.2 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory shootC = drive.trajectoryBuilder((intakeC.end()), false)
                .back(20.0)
                .build();
        Trajectory intakeC2 = drive.trajectoryBuilder((shootC.end()), false)
                .lineToConstantHeading(new Vector2d(-34, -38),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(0.35 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory shootC2 = drive.trajectoryBuilder((intakeC2.end()), false)
                .lineToConstantHeading(new Vector2d(-2, -43))
                .build();
        Trajectory deliverC1 = drive.trajectoryBuilder((shootC2.end()), false)
                .lineToConstantHeading(new Vector2d(48.0, -60.5))
                .build();
        Trajectory runWobbleC = drive.trajectoryBuilder((deliverC1.end()), false)
                .splineToSplineHeading(new Pose2d(-29.0, -22.0, Math.toRadians(0.0)), Math.toRadians(180.0)) //-31, -26
                .build();
        Trajectory wobbleSlowC = drive.trajectoryBuilder((runWobbleC.end()), false)
                .back(10, //6
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(0.3 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory deliverC2 = drive.trajectoryBuilder((wobbleSlowC.end()), false)
                //.splineToLinearHeading(new Pose2d(30, -69.5, Math.toRadians(180.0)), Math.toRadians(0.0))
                .lineToLinearHeading(new Pose2d(30, -75.5, Math.toRadians(180)))
                .build();
        Trajectory parkC = drive.trajectoryBuilder((deliverC2.end()), false)
                .lineToConstantHeading(new Vector2d(0, -60))
                .build();

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setPower(1); //start shooter
        shooterMotor2.setPower(1);
        //navigate to launch line
        drive.followTrajectory(shootTrajectory);
        //shoot pre-loaded rings
        sleep(250);
        for (int i = 0; i < 3; ++i) {
            shooterServo.setPosition(0.15);
            sleep(250);
            shooterServo.setPosition(0);
            sleep(250);
        }
        shooterMotor.setPower(0);
        shooterMotor2.setPower(0);

        switch (box) {
            case 0: // navigate to a, deposit, and go to pick up wobble 2
                wobbleGoal.lowerArm();
                drive.followTrajectory(deliverA1);
                wobbleGoal.release();
                drive.followTrajectory(backawayA1);
                drive.followTrajectory(runWobbleA);
                drive.followTrajectory(wobbleSlowA);
                break;
            case 1: // intake 1, shoot 1, navigate to b, deposit, and go to pick up wobble 2
                intake.togglePower();
                sleep(250);
                drive.followTrajectory(intakeB);
                shooterMotor.setPower(1);
                shooterMotor2.setPower(1);
                drive.followTrajectory(shootB);
                sleep(250);
                intake.togglePower();
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.15);
                    sleep(250);
                    shooterServo.setPosition(0);
                    sleep(300);
                }
                shooterMotor.setPower(0);
                shooterMotor2.setPower(0);
                wobbleGoal.lowerArm();
                drive.followTrajectory(deliverB1);
                wobbleGoal.release();
                drive.followTrajectory(runWobbleB);
                drive.followTrajectory(wobbleSlowB);
                break;

            case 2: // intake 3, shoot 3, intake 1, shoot 1, navigate to spot c, deposit, and go to pick up wobble 2
                intake.togglePower();
                drive.followTrajectory(strafeC);
                drive.followTrajectory(intakeC);
                shooterMotor.setPower(1);
                shooterMotor2.setPower(1);
                drive.followTrajectory(shootC);
                intake.togglePower();
                sleep(250);
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.15);
                    sleep(250);
                    shooterServo.setPosition(0);
                    sleep(300);
                }
                shooterMotor.setPower(0);
                shooterMotor2.setPower(0);
                intake.togglePower();
                drive.followTrajectory(intakeC2);
                shooterMotor.setPower(1);
                shooterMotor2.setPower(1);
                drive.followTrajectory(shootC2);
                sleep(100);
                intake.togglePower();
                sleep(100);
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.15);
                    sleep(250);
                    shooterServo.setPosition(0);
                    sleep(300);
                }
                shooterMotor.setPower(0);
                shooterMotor2.setPower(0);
                wobbleGoal.lowerArm();
                drive.followTrajectory(deliverC1);
                wobbleGoal.release();
                drive.followTrajectory(runWobbleC);
                drive.followTrajectory(wobbleSlowC);
                break;
        }
        wobbleGoal.grab();     // wobble 2 grabbbbb
        sleep(250);

        switch (box) {  // deliver wobble 2 and park
            case 0: //deposit second wobble at a
                drive.followTrajectory(deliverA2);
                wobbleGoal.release();
                sleep(200);
                drive.followTrajectory(parkA);
                break;
            case 1: //deposit second wobble at b
                drive.followTrajectory(deliverB2);
                wobbleGoal.release();
                sleep(250);
                drive.followTrajectory(parkB);
                break;
            case 2: //deposit second wobble at c
                drive.followTrajectory(deliverC2);
                wobbleGoal.release();
                drive.followTrajectory(parkC);
                break;
        }
    }
}