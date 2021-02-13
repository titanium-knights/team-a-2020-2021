package org.firstinspires.ftc.teamcode.util.trex;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
import org.firstinspires.ftc.teamcode.utils.trex.MDrive;

@Autonomous(name = "scuffed teleop lol")

public class ChrisTeleop extends LinearOpMode {
    MDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MDrive(HardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;

            if ((x > 0) && (y == 0)) {
                drive.driveXYRot(0.5, 0, 0);
            } else if ((x > 0) && (y > 0)) {
                drive.driveXYRot(0.5, 0.5, 0);
            } else if ((x > 0) && (y < 0)) {
                drive.driveXYRot(0.5, -0.5, 0);
            } else if ((x == 0) && (y == 0)) {
                drive.driveXYRot(0, 0, 0);
            } else if ((x == 0) && (y > 0)) {
                drive.driveXYRot(0, 0.5, 0);
            }
        }
    }
}

*/