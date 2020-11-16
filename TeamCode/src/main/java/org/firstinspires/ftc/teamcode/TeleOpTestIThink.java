package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "name")
    public class TeleOpTestIThink extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor left  = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()) {
            double speed_stick = gamepad1.right_stick_y;
            double turning = gamepad1.left_stick_x;

            left.setPower(Math.min(Math.max(speed_stick-turning, -1), 1));
            right.setPower(Math.min(Math.max(speed_stick+turning, -1), 1));

        }

    }

}
