package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.GamepadManager;
import org.firstinspires.ftc.teamcode.util.WobbleGoal;

@TeleOp(name = "Wobble Goal Tester")
public class WobbleGoalTest extends OpMode {
    GamepadManager gm;
    WobbleGoal goal;

    @Override public void init() {
        gm = new GamepadManager(gamepad1);
        goal = WobbleGoal.standard(hardwareMap);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override public void loop() {
        if (gamepad1.dpad_up) {
            goal.liftArm();
        } else if (gamepad1.dpad_down) {
            goal.lowerArm();
        } else {
            goal.stopArm();
        }

    }
}