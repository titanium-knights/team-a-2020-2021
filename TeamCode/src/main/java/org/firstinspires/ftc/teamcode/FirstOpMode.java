package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class FirstOpMode extends LinearOpMode{
    public void runOpMode() {
        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");

        waitForStart();

        left.setPower(1);
        right.setPower(1);

        sleep(1000);

        left.setPower(0);
        right.setPower(0);

        sleep(1000);

        left.setPower(-1);
        right.setPower(-1);
        sleep(2345);
    }
}
