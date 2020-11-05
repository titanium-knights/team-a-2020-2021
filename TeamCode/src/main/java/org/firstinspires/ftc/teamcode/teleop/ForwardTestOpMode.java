package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

@TeleOp(name = "Forward Test")
public class ForwardTestOpMode extends OpMode {
    private final ArrayList<DcMotor> motors = new ArrayList<>();

    @Override
    public void init() {
        motors.add(hardwareMap.dcMotor.get("mecanum_fl"));
        motors.add(hardwareMap.dcMotor.get("mecanum_fr"));
        motors.add(hardwareMap.dcMotor.get("mecanum_bl"));
        motors.add(hardwareMap.dcMotor.get("mecanum_br"));

        motors.get(0).setDirection(DcMotorSimple.Direction.FORWARD);
        motors.get(1).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(2).setDirection(DcMotorSimple.Direction.FORWARD);
        motors.get(3).setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void loop() {
        for (DcMotor motor: motors) {
            motor.setPower(gamepad1.dpad_up ? 0.5 : 0.0);
        }
    }
}
