package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoal {
    public CRServo arm;
    public Servo grabber;

    public WobbleGoal(CRServo arm, Servo grabber) {
        this.arm = arm;
        this.grabber = grabber;
    }

    public void release() {
        this.grabber.setPosition(1);
    }

    public void grab() {
        this.grabber.setPosition(0);
    }

    public void stopGrabber() {}

    public void liftArm() {
        this.arm.setPower(1);
    }

    public void lowerArm() {
        this.arm.setPower(-1);
    }

    public void stopArm() {
        this.arm.setPower(0);
    }

    public static WobbleGoal standard(HardwareMap hardwareMap) {
        return new WobbleGoal(hardwareMap.crservo.get("wobble_arm"), hardwareMap.servo.get("wobble"));
    }
}
