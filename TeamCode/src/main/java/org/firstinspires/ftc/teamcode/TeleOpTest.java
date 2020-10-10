package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tele Op Test", group = "Tests & Experiments")
public class TeleOpTest extends LinearOpMode {
    @Override public void runOpMode() {
        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");

        waitForStart();

        while (opModeIsActive()) {
            double stick = gamepad1.left_stick_x;
            left.setPower(stick);

            if (gamepad2.x){
                right.setPower(1);
            } else{
                right.setPower(0);
            }
        }
    }
}
