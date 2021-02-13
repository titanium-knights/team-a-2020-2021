package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    @Override public void loop() {
        gm.updateAll();
        if (gamepad1.dpad_right) {
            goal.grab();
        } else if (gamepad1.dpad_left) {
            goal.release();
        }

        if (gm.dpad_up.pressed()) {
            goal.grabber.setPosition(goal.grabber.getPosition() + 0.1);
        } else if (gm.dpad_down.pressed()) {
            goal.grabber.setPosition(goal.grabber.getPosition() - 0.1);
        }

        telemetry.addData("Pos", goal.grabber.getPosition());
    }
}
