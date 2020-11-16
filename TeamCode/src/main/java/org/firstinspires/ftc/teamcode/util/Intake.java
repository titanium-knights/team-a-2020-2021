package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private static final double MAX_POWER = 1;

    Motor top;
    Motor bottom;
    boolean on;
    double power = MAX_POWER;

    public static class Motor {
        String name = null;
        DcMotor motor;

        public Motor(DcMotor motor, String name) {
            this.motor = motor;
            this.name = name;
        }
    }

    public Intake (Motor top, Motor bottom) {
        this.top = top;
        this.bottom = bottom;
        this.top.motor.setDirection(DcMotor.Direction.FORWARD);
        this.bottom.motor.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Sets the pivot flywheels' speeds to their saved power.
     */
    public void togglePower () {
        on = !on;
        this.top.motor.setPower(on ? power : 0);
        this.bottom.motor.setPower(on ? power : 0);
    }

    /**
     * Sets the direction of the flywheels.
     */
    public void toggleDirection () {
        power = -power;
        this.top.motor.setPower(power);
        this.bottom.motor.setPower(power);
    }

    /**
     * Sets the pivot flywheels' speeds to 0.
     */
    public void stop () {
        this.top.motor.setPower(0);
        this.bottom.motor.setPower(0);
    }

    public static Intake standard(HardwareMap hardwareMap) {
        Motor top = new Motor(hardwareMap.get(DcMotor.class, "intake_top"), "intake_top");
        Motor bottom  = new Motor(hardwareMap.get(DcMotor.class, "intake_bottom"), "intake_bottom");
        return new Intake(top, bottom);
    }

}