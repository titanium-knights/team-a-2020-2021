package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.teamcode.util.*;

@TeleOp(name = "MecanumTest", group = "Tests B Experiments")
public class MecanumTest extends LinearOpMode {

    private static final double JOYSTICK_SENSITIVITY = 0.2f;

    private MecanumDrive drive;
    private VectoringTest VectoringTest;
    private boolean slowMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = MecanumDrive.standard(hardwareMap);
        for (DcMotor motor : drive.getMotors()) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        VectoringTest = VectoringTest.standard(hardwareMap);
        CRServo Left = hardwareMap.crservo.get("Left_Vector");
        CRServo Right = hardwareMap.crservo.get("Right_Vector");

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

            // Drives in the inputted direction.
            MecanumDrive.Motor.Vector2D vector = new MecanumDrive.Motor.Vector2D(strafe, speed);
            drive.move(slowMode ? 0.3 : 1.0, vector, turn * (slowMode ? 0.3 : 1.0));
            if (gm1.left_stick_button.pressed()) slowMode = !slowMode;
            telemetry.addData("Slow Mode", slowMode ? "Yes" : "No");
            telemetry.update();

            if (gamepad1.dpad_up) {
                VectoringTest.RunVectorsWithPower(Right, Left);
            } else if (gamepad1.dpad_down) {
                VectoringTest.ReverseVectorsWithPower(Right, Left);
            } else {
                VectoringTest.StopVectors(Right,Left);

                gm1.updateAll();
                gm2.updateAll();
            }
        }
    }
}