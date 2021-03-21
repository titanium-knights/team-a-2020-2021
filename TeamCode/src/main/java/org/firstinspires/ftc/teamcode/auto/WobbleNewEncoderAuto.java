package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.WobbleGoal;

@Autonomous(name = "Wole New Encoder Auto")
public class WobbleNewEncoderAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        WobbleGoal wobble = WobbleGoal.standard(hardwareMap);

        wobble.lowerArm();
        while (wobble.getPosition() > -2521) {
            sleep(10);
        }
        wobble.stopArm();
        sleep(3000);

        wobble.lowerArm();
        while (wobble.getPosition() > -6069) {
            sleep(10);
        }
        wobble.stopArm();
        sleep(3000);

        wobble.liftArm();
        while (wobble.getPosition() < 0) {
            sleep(10);
        }
        wobble.stopArm();
        sleep(3000);

    }
}
