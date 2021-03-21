package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.WobbleGoal;

public class WobbleNewEncoderAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        WobbleGoal wobble = WobbleGoal.standard(hardwareMap);

        wobble.lowerArm();
        while (wobble.getPosition() > -2521) {
            sleep(10);
        }
        sleep(3000);

        wobble.lowerArm();
        while (wobble.getPosition() > -6069) {
            sleep(10);
        }
        sleep(3000);

        wobble.liftArm();
        while (wobble.getPosition() < 0) {
            sleep(10);
        }
        sleep(3000);

        wobble.stopArm();
    }
}
