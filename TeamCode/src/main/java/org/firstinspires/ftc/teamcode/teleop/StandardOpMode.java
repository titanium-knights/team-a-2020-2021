package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.*;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Shooter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "Standard Tele Op", group = "Tests B Experiments")
public class StandardOpMode extends LinearOpMode {

    public enum RobotState {
        INDEXING,
        HIGH,
        WOBBLE,
    }

    private static final double JOYSTICK_SENSITIVITY = 0.0f;
    private MecanumDrive drive;
    private Intake intake;
    private Shooter shooter;
    private boolean slowMode = false;
    private WobbleGoal wobbleGoal;
    private ElapsedTime runtime = new ElapsedTime();
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
        shooter = Shooter.standard(hardwareMap);
        SampleMecanumDrive mecDrive = new SampleMecanumDrive(hardwareMap);

        // TODO: Make utility class - shooter.shoot() or something?
        DcMotorEx shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotor shooterMotor2 = hardwareMap.dcMotor.get("shooter2");
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
                .strafeLeft(-23) //make negative for BLUE, positive for RED
                .build();

        RobotState robotState = RobotState.INDEXING;

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
            double curVelo = shooterMotor1.getVelocity() / 28 * .098425 * Math.PI;
            // Drives in the inputted direction.
            MecanumDrive.Motor.Vector2D vector = new MecanumDrive.Motor.Vector2D(strafe, speed);
            drive.move(slowMode ? 0.3 : 1.0, vector, turn * (slowMode ? 0.5 : 1.0));
            if (gm1.left_stick_button.pressed()) slowMode = !slowMode;
            telemetry.addData("Slow Mode", slowMode ? "Yes" : "No");
            telemetry.update();

            if (gamepad1.dpad_left) {
                MecDrive.setPoseEstimate(startPose);
            }
            if (gamepad1.dpad_right) {
                //paths yes
                shooterServo.setPosition(0);
                shooterFlap.setPosition(0.02);
                shooterMotor1.setPower(0.8);
                shooterMotor2.setPower(0.8);
                MecDrive.followTrajectory(PS1);
                sleep(250);
                shooterServo.setPosition(0.12);
                sleep(250);
                shooterServo.setPosition(0.00);
                mecDrive.turn(Math.toRadians(8));
                sleep(200);
                shooterServo.setPosition(0.12);
                sleep(250);
                shooterServo.setPosition(0.00);
                mecDrive.turn(Math.toRadians(-15));
                sleep(200);
                shooterServo.setPosition(0.12);
                sleep(250);
                shooterServo.setPosition(0.00);
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
            }

            switch (robotState) {
                case INDEXING:
                    intake.intakeOn();
                    if (gm1.left_bumper.pressed()) {
                        intake.toggleDirection();
                    }
                    if (gm1.right_bumper.pressed()) {
                        intake.togglePower();
                    }
                    shooterServo.setPosition(shooter.leftPusherPos);
                    break;
                case HIGH:
                    shooter.fireSpeed = 22;
                    //shooter.toggleShooterPower(curVelo, runtime.seconds());
                    shooter.shooterOn(curVelo,runtime.seconds());
                    if (gamepad1.right_bumper && shooter.shots < 3) {
                        shooter.timedFireN(curVelo);
                    } else if (shooter.shots < 3) {
                        shooterServo.setPosition(shooter.leftPusherPos);
                        shooter.timedCancel();
                    } else {
                        robotState = RobotState.INDEXING;
                    }
                    break;
                case WOBBLE:
                    if (gamepad1.dpad_up) {
                        wobbleGoal.grab();
                        sleep(400);
                        raiseUp();
                    } else if (gamepad1.dpad_down) {
                        lowerArm();
                        sleep(200);
                        wobbleGoal.release();
                    } else {
                        wobbleGoal.stopArm();
                    }
                    break;
            }

            FtcDashboard dashboard = FtcDashboard.getInstance();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Velocity", shooter.fireSpeed);
            packet.put("Actual Velocity Shooter", curVelo);
            dashboard.sendTelemetryPacket(packet);

            // Toggles power and direction of the intake motors.
            //if (gm1.left_bumper.pressed()&&robotState == RobotState.INDEXING) intake.toggleDirection();
            if (robotState!= RobotState.HIGH) {
                shooter.safetySwitch();
            }
            if(gm1.a.pressed()) {
                robotState = RobotState.WOBBLE;
            }
            if(gm1.b.pressed()) {
                intake.togglePower();
                shooterFlap.setPosition(0.016);
                robotState = RobotState.HIGH;
            }
            if(gm1.x.pressed()) {
                shooterMotor1.setPower(0);
                shooterMotor1.setPower(0);
                robotState = RobotState.INDEXING;
            }
            if (gm1.x.pressed()) {
                shooter.toggleShooterPower(curVelo, runtime.seconds());
                intake.togglePower();
            }

            if (gm1.b.pressed())
                shooter.timedFireN(curVelo);

            /*   I'm pretty sure this has been causing some shooter issues.
            if (shooterIsShooting) shooterMotor1.setPower(1);
            else shooterMotor1.setPower(0);
            if (shooterIsShooting) shooterMotor2.setPower(1);
            else shooterMotor2.setPower(0);
            */
            /*
            // Todo: Uncomment this and remove the old code when you want to test the Shooter class
            // Toggle shooter power and controls the pinball as necessary.
            if (gm1.x.pressed()) shooter.toggleShooterPower();
            shooter.nudgeRings(gamepad1.y, gamepad1.a);
             */
            // Update the GamepadManagers (should happen at the end or beginning of the loop
        }
    }
}
