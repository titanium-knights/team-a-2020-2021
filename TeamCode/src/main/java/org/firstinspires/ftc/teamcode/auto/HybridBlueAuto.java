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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Shooter;
import org.firstinspires.ftc.teamcode.util.WobbleGoal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Arrays;

@Autonomous(name = "HybridBlueAuto")
public class HybridBlueAuto extends LinearOpMode {

    //random vision stuff
    private static final int CAMERA_WIDTH = 640; // width of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution
    public static int HORIZON = 100; // horizon value to tune
    private static final boolean DEBUG = true; // if debug is wanted, change to true
    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam
    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;
    private Shooter shooter;
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        WobbleGoal wobbleGoal = WobbleGoal.standard(hardwareMap);
        DcMotorEx shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotor shooterMotor = hardwareMap.dcMotor.get("shooter1");
        Intake intake = Intake.standard(hardwareMap);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor shooterMotor2 = hardwareMap.dcMotor.get("shooter2");
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter = Shooter.standard(hardwareMap);
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
        flap.setPosition(0.035); //High goal: 0.00      PS: 0.05
        shooterServo.setPosition(0.00);

        int box = 0; // 0 is for closest box, 1 is for the middle box, 2 is for the farthest box
        UGContourRingPipeline.Height height = pipeline.getHeight();
        if (height == UGContourRingPipeline.Height.FOUR) {
            box = 2;
        } else if (height == UGContourRingPipeline.Height.ONE) {
            box = 1;}

        waitForStart();
        double curVelo = shooterMotor1.getVelocity() / 28 * .098425 * Math.PI;

        //defines start pose
        Pose2d startPose = new Pose2d (-64, 18.0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        Trajectory shootTrajectory = drive.trajectoryBuilder((startPose), true)
                .splineToConstantHeading(new Vector2d(-1.0, 10.0), Math.toRadians(0))
                .build();
        Trajectory BounceBackPath1 = drive.trajectoryBuilder((shootTrajectory.end().plus(new Pose2d(0, 0, Math.toRadians(180)))), false)
                .lineToLinearHeading(new Pose2d(54.0, -12.0, Math.toRadians(0)),
        new MinVelocityConstraint(
                Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(0.8 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(1, ()->{
                    intake.togglePower();
                    shooterMotor.setPower(0);
                    shooterMotor2.setPower(0);
                })
                .build();
        Trajectory BounceBackPath2 = drive.trajectoryBuilder(BounceBackPath1.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .lineToLinearHeading(new Pose2d(52.0, 54.0, Math.toRadians(90)),
                    new MinVelocityConstraint(
                    Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(0.6 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(2, () -> {
                    wobbleGoal.lowerArm();
                })
                .build();
        Trajectory deliverA = drive.trajectoryBuilder((BounceBackPath2.end()), false)
                .lineToLinearHeading(new Pose2d(20.0, 60.0, Math.toRadians(0.0)))
                .build();
        Trajectory avoidA = drive.trajectoryBuilder(deliverA.end(), false)
                .forward(8.0)
                .addDisplacementMarker(5, ()-> {
                    wobbleGoal.liftArm();
                    wobbleGoal.grab();
                })
                .build();
        Trajectory no = drive.trajectoryBuilder((avoidA.end()), false)
                .lineToLinearHeading(new Pose2d(16.0, 16.0, Math.toRadians(0.0)))
                .build();
        Trajectory shootBounceBackA = drive.trajectoryBuilder((no.end()), false)
                .lineToLinearHeading(new Pose2d(-5.0, 9.0, Math.toRadians(198.0)))
                .addDisplacementMarker(2, ()->{
                    flap.setPosition(0.00); //shoot bouncebacks into high goal
                    wobbleGoal.liftArm();
                    wobbleGoal.grab();
                })
                .build();

        Trajectory deliverB = drive.trajectoryBuilder((BounceBackPath2.end()), false)
                .splineToLinearHeading(new Pose2d(34.0, 16.0, Math.toRadians(-90.0)), Math.toRadians(180.0))
                .build();
        Trajectory avoidB = drive.trajectoryBuilder((deliverB.end()), false)
                .forward(4)
                .build();
        Trajectory shootBounceBackB = drive.trajectoryBuilder((avoidB.end()), false)
                .addDisplacementMarker(2, ()->{
                    flap.setPosition(0.00); //shoot bouncebacks into high goal
                    wobbleGoal.liftArm();
                    wobbleGoal.grab();
                })
                .lineToLinearHeading(new Pose2d(-5.0, 9.0, Math.toRadians(198.0)))
                .build();

        Trajectory deliverC = drive.trajectoryBuilder((BounceBackPath2.end()), false)
                .lineToLinearHeading(new Pose2d(48.0, 40.0, Math.toRadians(-90.0)))
                .build();
        Trajectory avoidC = drive.trajectoryBuilder((deliverC.end()), false)
                .forward(8)
                .build();
        Trajectory shootBounceBackC = drive.trajectoryBuilder((avoidC.end()), false)
                .lineToLinearHeading(new Pose2d(-5.0, 9.0, Math.toRadians(198.0)))
                .addDisplacementMarker(2, ()->{
                    flap.setPosition(0.00); //shoot bouncebacks into high goal
                    wobbleGoal.liftArm();
                    wobbleGoal.grab();
                })
                .build();

        Trajectory park = drive.trajectoryBuilder((shootBounceBackA.end()), false)
                .lineToLinearHeading(new Pose2d(12.0, 8.0, Math.toRadians(180)))
                .build();


        flap.setPosition(0.017); //High goal: 0.00      PS: 0.05
        shooter.fireSpeed = 17;
        shooter.toggleShooterPower(curVelo,runtime.seconds());
        drive.followTrajectory(shootTrajectory);
        sleep(250);
        while (!shooter.atSpeed(curVelo)) {
            curVelo = shooterMotor1.getVelocity() / 28 * .098425 * Math.PI;
            shooter.shooterOn(curVelo,runtime.seconds());
            shooterServo.setPosition(.12);
        }
        sleep(300);
        flap.setPosition(0.017); //High goal: 0.00      PS: 0.05
        drive.turn(Math.toRadians(8));
        shooterServo.setPosition(0);
        sleep(300);
        while (!shooter.atSpeed(curVelo)) {
            curVelo = shooterMotor1.getVelocity() / 28 * .098425 * Math.PI;
            shooter.shooterOn(curVelo,runtime.seconds());
            shooterServo.setPosition(.12);
        }
        sleep(250);
        drive.turn(Math.toRadians(-15));
        shooterServo.setPosition(0);
        sleep(250);
        while (!shooter.atSpeed(curVelo)) {
            curVelo = shooterMotor1.getVelocity() / 28 * .098425 * Math.PI;
            shooter.shooterOn(curVelo,runtime.seconds());
            shooterServo.setPosition(.12);
        }
        sleep(250);
        shooterServo.setPosition(0);
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(BounceBackPath1);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(BounceBackPath2);

        switch (box) {  // deliver wobble 2 and park
            case 0: //deposit second wobble at a
                drive.followTrajectory(deliverA);
                wobbleGoal.release();
                drive.followTrajectory(avoidA);
                drive.followTrajectory(no);
                drive.followTrajectory(shootBounceBackA);
                break;
            case 1: //deposit second wobble at b
                drive.followTrajectory(deliverB);
                wobbleGoal.release();
                drive.followTrajectory(avoidB);
                drive.followTrajectory(shootBounceBackB);
                break;
            case 2: //deposit second wobble at c
                drive.followTrajectory(deliverC);
                wobbleGoal.release();
                sleep(50);
                drive.followTrajectory(avoidC);
                drive.followTrajectory(shootBounceBackC);
                break;
        }
        shooter.fireSpeed = 21;
        shooter.toggleShooterPower(curVelo, runtime.seconds());
        intake.togglePower();
        sleep(500);
        for (int i = 0; i < 3; ++i) {
            if (shooter.atSpeed(curVelo)) {
                shooterServo.setPosition(0.12);
                sleep(100);
                shooterServo.setPosition(0.00);
            }
        }
        wobbleGoal.lowerArm();
        wobbleGoal.release();
        sleep(100);
        drive.followTrajectory(park);

    }
}