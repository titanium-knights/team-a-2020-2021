package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OkAnli extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left  = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");

        waitForStart();

        left.setPower(1);
        right.setPower(1);

        sleep(15245);

        left.setPower(0);
        right.setPower(0);

        sleep(7986);
    }
}
