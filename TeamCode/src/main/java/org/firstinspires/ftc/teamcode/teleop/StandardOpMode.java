package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.MecanumDrive;

@TeleOp(name = "Tele Op Test", group = "Tests B Experiments")
public class StandardOpMode extends LinearOpMode {

    private static final double JOYSTICK_SENSITIVITY = 0.2f;

    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = MecanumDrive.standard(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // Gets the speed, strafe, and turn of the robot and accounts for stick drifting
            double speed = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            if (Math.abs(turn) < JOYSTICK_SENSITIVITY) turn = 0;
            if (Math.abs(strafe) < JOYSTICK_SENSITIVITY) strafe = 0;
            if (Math.abs(speed) < JOYSTICK_SENSITIVITY) speed = 0;

            // Drives in the inputted direction.
            MecanumDrive.Motor.Vector2D vector = new MecanumDrive.Motor.Vector2D(strafe, speed);
            drive.move(1, vector, turn);

            // Toggles power and direction of the intake motors.
            if (gm1.left_bumper.pressed()) intake.togglePower();
            if (gm1.right_bumper.pressed()) intake.toggleDirection();
            if (gm1.x.pressed()) shooterIsShooting = !shooterIsShooting;
            if (gamepad1.a) {
                shooterServo.setPower(-0.5);
            } else if (gamepad1.y) {
                shooterServo.setPower(0.5);
            } else {
                shooterServo.setPower(0);
            }
            if (gamepad1.dpad_up) {
                wobble.liftArm();
            } else if (gamepad1.dpad_down) {
                wobble.lowerArm();
            } else {
                wobble.stopArm();
            }
            if (gamepad1.dpad_left) {
                wobble.release();
            } else if (gamepad1.dpad_right) {
                wobble.grab();
            } else {
                wobble.stopGrabber();
            }

            if (shooterIsShooting) shooterMotor.setPower(-1); else shooterMotor.setPower(0);


            // Todo: Uncomment this and remove the old code when you want to test the Shooter class
            // Toggle shooter power and controls the pinball as necessary.
            if (gm1.x.pressed()) shooter.toggleShooterPower();
            shooter.nudgeRings(gamepad1.y, gamepad1.a);


            // Update the GamepadManagers (should happen at the end or beginning of the loop)
            gm1.updateAll();
            gm2.updateAll();

        }
    }
}