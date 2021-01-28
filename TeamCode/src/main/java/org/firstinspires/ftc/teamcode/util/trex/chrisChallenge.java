package org.firstinspires.ftc.teamcode.util.trex;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous(name = "Chris Challenge Part Two")
public class chrisChallenge extends LinearOpMode{
    MDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Mdrive(HardwareMapardwareMap);
        waitForStart();

        drive.driveForwardWithPower(1);
        sleep(1000);

        drive.strafeLeftWithPower(1);
        sleep(1000);

        drive.driveBackwardWithPower(1);
        sleep(1000);

        drive.strafeRightWithPower(1);
        sleep(1000);
    }
}
