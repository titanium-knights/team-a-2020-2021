package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WobbleGoal {
    public CRServo arm;
    public CRServo grabber;

    public WobbleGoal(CRServo arm, CRServo grabber) {
        this.arm = arm;
        this.grabber = grabber;
    }

    public void release() {
        this.grabber.setPower(0.5);
    }

    public void grab() {
        this.grabber.setPower(-0.5);
    }

    public void stopGrabber() {
        this.grabber.setPower(0);
    }

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
        return new WobbleGoal(hardwareMap.crservo.get("wobble_arm"), null);
    }
}
