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

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        WobbleGoal wobbleGoal = WobbleGoal.standard(hardwareMap);
        DcMotor shooterMotor = hardwareMap.dcMotor.get("shooter1");
        Intake intake = Intake.standard(hardwareMap);
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
        flap.setPosition(0.035); //High goal: 0.00      PS: 0.05
        shooterServo.setPosition(0.00);

        int box = 0; // 0 is for closest box, 1 is for the middle box, 2 is for the farthest box
        UGContourRingPipeline.Height height = pipeline.getHeight();
        if (height == UGContourRingPipeline.Height.FOUR) {
            box = 2;
        } else if (height == UGContourRingPipeline.Height.ONE) {
            box = 1;}

        waitForStart();

        //defines start pose
        Pose2d startPose = new Pose2d (-64, 18.0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        Trajectory shootTrajectory = drive.trajectoryBuilder((startPose), true)
                .splineToConstantHeading(new Vector2d(0.0, 10.0), Math.toRadians(0))
                .build();
        Trajectory BounceBackPath1 = drive.trajectoryBuilder((shootTrajectory.end()), false)
                .lineToLinearHeading(new Pose2d(56.0, 0.0, Math.toRadians(0)))
                .build();
        Trajectory BounceBackPath2 = drive.trajectoryBuilder(BounceBackPath1.end(), false)
                .lineToLinearHeading(new Pose2d(56.0, 54.0, Math.toRadians(0)),
                    new MinVelocityConstraint(
                    Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        flap.setPosition(0.018); //High goal: 0.00      PS: 0.05
        shooterMotor.setPower(0.69);
        shooterMotor2.setPower(0.69);
        drive.followTrajectory(shootTrajectory);
        sleep(400);
        shooterServo.setPosition(0.15);
        sleep(200);
        drive.turn(Math.toRadians(8));
        shooterServo.setPosition(0);
        flap.setPosition(0.017); //High goal: 0.00      PS: 0.05
        sleep(400);
        shooterServo.setPosition(0.15);
        sleep(200);
        drive.turn(Math.toRadians(-15));
        shooterServo.setPosition(0);
        sleep(400);
        shooterServo.setPosition(0.15);
        sleep(200);
        shooterServo.setPosition(0);
        drive.turn(Math.toRadians(0));
        shooterMotor.setPower(0);
        shooterMotor2.setPower(0);
        intake.togglePower();
        sleep(200);
        drive.followTrajectory(BounceBackPath1);
        sleep(100);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(BounceBackPath2);
    }
}