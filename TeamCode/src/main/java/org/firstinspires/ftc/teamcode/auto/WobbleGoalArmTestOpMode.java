package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Wobble Goal Claw Test Op Mode")
public class WobbleGoalArmTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo claw = hardwareMap.servo.get("wobble");

        waitForStart();

        claw.setPosition(0);
        sleep(1000);
        claw.setPosition(1);
        sleep(1000);
    }
}
