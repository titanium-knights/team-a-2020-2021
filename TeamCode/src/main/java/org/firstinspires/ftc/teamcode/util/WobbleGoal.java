package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class WobbleGoal {
    public Servo grabber;
    public DcMotor arm;

    public WobbleGoal(Servo grabber, DcMotor arm) {
        this.grabber = grabber;
        this.arm = arm;

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void release() {
        this.grabber.setPosition(0.5);
    }
 
    public void grab() {
        this.grabber.setPosition(0.05);
    }

    public void stopGrabber() {}

    public void liftArm() {
        arm.setPower(0.6);
    }

    public void lowerArm() {
        arm.setPower(-0.6);
    }

    public void stopArm() {
        arm.setPower(0);
    }

    public static WobbleGoal standard(HardwareMap hardwareMap) {
        return new WobbleGoal(hardwareMap.servo.get("wobble"), hardwareMap.dcMotor.get("wobble_arm"));
    }
}
