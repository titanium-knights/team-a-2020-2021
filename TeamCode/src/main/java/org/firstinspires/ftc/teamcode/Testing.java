package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Testing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = ardwareMap.dcMotor.get("right");

        waitForStart();

        // move forward

        left.setPower(1);
        right.setPower(1);

        sleep(1000);

        // comment

        left.setPower(0);
        right.setPower(0);

        sleep(1000);

        left.setPower(-1);
        right.setPower(-1);

        sleep(1000);
    }
}
