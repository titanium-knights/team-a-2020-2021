package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoal {
    public Servo grabber;

    public WobbleGoal(Servo grabber) {
        this.grabber = grabber;
    }

    public void release() {
        this.grabber.setPosition(1);
    }

    public void grab() {
        this.grabber.setPosition(0);
    }

    public void stopGrabber() {}

    @Deprecated public void liftArm() {
        throw new RuntimeException("the arm doesn't exist anymore");
    }

    @Deprecated public void lowerArm() {
        throw new RuntimeException("the arm doesn't exist anymore");
    }

    @Deprecated public void stopArm() {
        throw new RuntimeException("the arm doesn't exist anymore");
    }

    public static WobbleGoal standard(HardwareMap hardwareMap) {
        return new WobbleGoal(hardwareMap.servo.get("wobble"));
    }
}
