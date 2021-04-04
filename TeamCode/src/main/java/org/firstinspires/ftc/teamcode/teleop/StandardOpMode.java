package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Shooter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Standard Tele Op", group = "Tests B Experiments")
public class StandardOpMode extends LinearOpMode {

    private static final double JOYSTICK_SENSITIVITY = 0.2f;

    private MecanumDrive drive;
    private Intake intake;
    private Shooter shooter;
    private boolean slowMode = false;
    private WobbleGoal wobbleGoal;

    private void raiseUp() {
        wobbleGoal.liftArm();
    }

    private void lowerArm() {
        wobbleGoal.lowerArm();
    }

    WobbleGoal goal;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = MecanumDrive.standard(hardwareMap);
        for (DcMotor motor : drive.getMotors()) {
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
        SampleMecanumDrive MecDrive = new SampleMecanumDrive(hardwareMap);
        Servo shooterFlap = hardwareMap.servo.get("Flap");

        wobbleGoal = WobbleGoal.standard(hardwareMap);
        goal = WobbleGoal.standard(hardwareMap);

        boolean shooterIsShooting = false;
        boolean flapRaised = false;

        GamepadManager gm1 = new GamepadManager(gamepad1);

        waitForStart();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        Trajectory PS1 = MecDrive.trajectoryBuilder((startPose), false)
                .strafeLeft(19)
                .build();
        Trajectory PS2 = MecDrive.trajectoryBuilder((PS1.end()), false)
                .strafeLeft(8)
                .build();
        Trajectory PS3 = MecDrive.trajectoryBuilder((PS2.end()), false)
                .strafeLeft(8)
                .build();

        while (opModeIsActive()) {
            gm1.updateAll();
            // Gets the speed, strafe, and turn of the robot and accounts for stick drifting
            double speed = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x; //
            if (Math.abs(turn) < JOYSTICK_SENSITIVITY) turn = 0;
            if (Math.abs(strafe) < JOYSTICK_SENSITIVITY) strafe = 0;
            if (Math.abs(speed) < JOYSTICK_SENSITIVITY) speed = 0;

            if (gamepad1.left_trigger > 0.5) {
                turn -= 0.2;
            } else if (gamepad1.right_trigger > 0.5) {
                turn += 0.2;
            }

            // Drives in the inputted direction.
            MecanumDrive.Motor.Vector2D vector = new MecanumDrive.Motor.Vector2D(strafe, speed);
            drive.move(slowMode ? 0.3 : 1.0, vector, turn * (slowMode ? 0.5 : 1.0));
            if (gm1.left_stick_button.pressed()) slowMode = !slowMode;
            telemetry.addData("Slow Mode", slowMode ? "Yes" : "No");
            telemetry.update();

            // Toggles power and direction of the intake motors.
            if (gm1.left_bumper.pressed()) intake.toggleDirection();
            if (gm1.right_bumper.pressed()) intake.togglePower();
            if (gm1.x.pressed()) {
                shooterIsShooting = !shooterIsShooting;
                intake.togglePower();
            }
            if (gm1.b.pressed())
                for (int i = 0; i < 3; ++i) {
                    shooterServo.setPosition(0.15);
                    sleep(250);
                    shooterServo.setPosition(0);
                    sleep(250);
                }

            if (gm1.y.pressed()) flapRaised = !flapRaised;
            if (flapRaised) {
                shooterFlap.setPosition(0.05);
            } else {
                shooterFlap.setPosition(0);
            }


            if (gm1.a.pressed()) {
                shooterServo.setPosition(0.15);
                sleep(100);
                shooterServo.setPosition(0);
                sleep(100);
            }
            if (gamepad1.dpad_up) {
                wobbleGoal.grab();
                sleep(400);
                raiseUp();
            } else if (gamepad1.dpad_down) {
                lowerArm();
                sleep(400);
                wobbleGoal.release();
            } else {
                wobbleGoal.stopArm();
            }
            if (gamepad1.dpad_left) {
                MecDrive.setPoseEstimate(startPose);
            }

            if (gamepad1.dpad_right) {
                //paths yes

                shooterServo.setPosition(0);
                shooterFlap.setPosition(0.05);
                shooterMotor1.setPower(0.8);
                shooterMotor2.setPower(0.8);
                //PowerShot1
                MecDrive.followTrajectory(PS1);
                sleep(250);
                shooterServo.setPosition(0.15);
                sleep(250);
                shooterServo.setPosition(0.00);
                //PowerShot2
                MecDrive.followTrajectory(PS2);
                sleep(100);
                shooterServo.setPosition(0.15);
                sleep(250);
                shooterServo.setPosition(0.00);
                //PowerShot3
                MecDrive.followTrajectory(PS3);
                sleep(100);
                shooterServo.setPosition(0.15);
                sleep(250);
                shooterServo.setPosition(0.00);
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
            }

                if (shooterIsShooting) shooterMotor1.setPower(1);
                else shooterMotor1.setPower(0);
                if (shooterIsShooting) shooterMotor2.setPower(1);
                else shooterMotor2.setPower(0);

            /*
            // Todo: Uncomment this and remove the old code when you want to test the Shooter class
            // Toggle shooter power and controls the pinball as necessary.
            if (gm1.x.pressed()) shooter.toggleShooterPower();
            shooter.nudgeRings(gamepad1.y, gamepad1.a);
             */

                // Update the GamepadManagers (should happen at the end or beginning of the loop)
            }
        }
    }
