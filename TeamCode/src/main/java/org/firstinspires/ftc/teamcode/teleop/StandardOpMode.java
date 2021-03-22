package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;

@TeleOp(name = "Standard Tele Op", group = "Tests B Experiments")
public class StandardOpMode extends LinearOpMode {

    private static final double JOYSTICK_SENSITIVITY = 0.2f;

    private MecanumDrive drive;
    private Intake intake;
    private Shooter shooter;
    private boolean slowMode = false;

    public static double shooterP = 40;
    public static double shooterI = 0;
    public static double shooterD = 0;
    public static double shooterF = 14;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = MecanumDrive.standard(hardwareMap);
        for (DcMotor motor: drive.getMotors()) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intake = Intake.standard(hardwareMap);
        // shooter = Shooter.standard(hardwareMap);

        // TODO: Make utility class - shooter.shoot() or something?
        DcMotor shooterMotor1 = hardwareMap.dcMotor.get("shooter1");
        DcMotor shooterMotor2 = hardwareMap.dcMotor.get("shooter2");
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo shooterServo = hardwareMap.servo.get("pinball");
        Servo shooterFlap = hardwareMap.servo.get("flap");

        boolean shooterIsShooting = false;
        boolean flapRaised = false;

        WobbleGoal wobble = WobbleGoal.standard(hardwareMap);

        GamepadManager gm1 = new GamepadManager(gamepad1);
        GamepadManager gm2 = new GamepadManager(gamepad2);

        waitForStart();

        while (opModeIsActive()) {

            // Gets the speed, strafe, and turn of the robot and accounts for stick drifting
            double speed = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x; // TODO: Remove negative sign
            if (Math.abs(turn) < JOYSTICK_SENSITIVITY) turn = 0;
            if (Math.abs(strafe) < JOYSTICK_SENSITIVITY) strafe = 0;
            if (Math.abs(speed) < JOYSTICK_SENSITIVITY) speed = 0;

            if (gamepad1.left_trigger > 0.5) {
                turn += 0.3;
            } else if (gamepad1.right_trigger > 0.5) {
                turn -= 0.3;
            }

            // Drives in the inputted direction.
            MecanumDrive.Motor.Vector2D vector = new MecanumDrive.Motor.Vector2D(strafe, speed);
            drive.move(slowMode ? 0.3 : 1.0, vector, turn * (slowMode ? 0.5 : 1.0));
            if (gm1.left_stick_button.pressed()) slowMode = !slowMode;
            telemetry.addData("Slow Mode", slowMode ? "Yes" : "No");
            telemetry.addData("Arm Power", wobble.arm1.getPower());
            telemetry.update();

            // Toggles power and direction of the intake motors.
            if (gm1.left_bumper.pressed()) intake.toggleDirection();
            if (gm1.right_bumper.pressed()) intake.togglePower();
            if (gm1.x.pressed()) shooterIsShooting = !shooterIsShooting;

            if (gm1.b.pressed())
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.15);
                    sleep(250);
                    shooterServo.setPosition(0);
                    sleep(250);
            }

            if (gm1.y.pressed()) flapRaised = !flapRaised;
            if (flapRaised){
                shooterFlap.setPosition(0.05);
            }else{
                shooterFlap.setPosition(0);
            }
            if (gamepad1.dpad_left) {
                wobble.release();
            } else if (gamepad1.dpad_right) {
                wobble.grab();
            }
            if (gamepad1.dpad_up) {
                wobble.liftArm();
            } else if (gamepad1.dpad_down) {
                wobble.lowerArm();
            } else {
                wobble.stopArm();
            }

            PIDFCoefficients shooterPIDF = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            //shooterMotor1.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
            //Shooter.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

            if (shooterIsShooting) shooterMotor1.setPower(1); else shooterMotor1.setPower(0);
            if (shooterIsShooting) shooterMotor2.setPower(1); else shooterMotor2.setPower(0);

            /*
            // Todo: Uncomment this and remove the old code when you want to test the Shooter class
            // Toggle shooter power and controls the pinball as necessary.
            if (gm1.x.pressed()) shooter.toggleShooterPower();
            shooter.nudgeRings(gamepad1.y, gamepad1.a);
             */

            // Update the GamepadManagers (should happen at the end or beginning of the loop)
            gm1.updateAll();
            gm2.updateAll();

        }
    }
}