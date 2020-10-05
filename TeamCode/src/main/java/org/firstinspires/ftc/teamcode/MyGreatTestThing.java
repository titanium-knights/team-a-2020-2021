package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MyGreatTestThing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");

        waitForStart();

        // Drive forward

        left.setPower(1);
        right.setPower(1);

        sleep(1000);

        // etc

        left.setPower(0);
        right.setPower(0);

        sleep(500);

        left.setPower(-1);
        right.setPower(-1);

        sleep(2345);
    }
}
