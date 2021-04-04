package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Shooter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "autopsTEST", group = "Tests B Experiments")
public class autopsTEST extends LinearOpMode {

    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = MecanumDrive.standard(hardwareMap);
        for (DcMotor motor : drive.getMotors()) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        DcMotor shooterMotor = hardwareMap.dcMotor.get("shooter1");
        DcMotor shooterMotor2 = hardwareMap.dcMotor.get("shooter2");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo shooterServo = hardwareMap.servo.get("pinball");
        Servo shooterFlap = hardwareMap.servo.get("Flap");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        GamepadManager gm1 = new GamepadManager(gamepad1);

        //paths yes
        Pose2d startPose = new Pose2d(0, 16.0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        Trajectory PS1 = drive.trajectoryBuilder((startPose), false)
                .strafeLeft(20)
                .build();
        Trajectory PS2 = drive.trajectoryBuilder((PS1.end()), false)
                .strafeLeft(7.5)
                .build();
        Trajectory PS3 = drive.trajectoryBuilder((PS2.end()), false)
                .strafeLeft(7.5)
                .build();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_right) {
                shooterServo.setPosition(0);
                shooterFlap.setPosition(0.05);
                shooterMotor.setPower(1);
                shooterMotor2.setPower(1);
                //PowerShot1
                drive.followTrajectory(PS1);
                sleep(250);
                shooterServo.setPosition(0.15);
                sleep(250);
                shooterServo.setPosition(0.00);
                sleep(250);
                //PowerShot2
                drive.followTrajectory(PS2);
                sleep(100);
                shooterServo.setPosition(0.15);
                sleep(250);
                shooterServo.setPosition(0.00);
                sleep(250);
                //PowerShot3
                drive.followTrajectory(PS3);
                sleep(100);
                shooterServo.setPosition(0.15);
                sleep(250);
                shooterServo.setPosition(0.00);
                sleep(250);
                shooterMotor.setPower(0);
                shooterMotor2.setPower(0);
            }
        }
        gm1.updateAll();
    }
}


