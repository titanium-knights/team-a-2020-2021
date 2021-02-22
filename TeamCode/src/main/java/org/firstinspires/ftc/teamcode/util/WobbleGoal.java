package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoal {
    public Servo grabber;
    public DcMotor arm;

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

    public void liftArm() {
        arm.setPower(0.15);
    }

    public void lowerArm() {
        arm.setPower(-0.15);
    }

    public void stopArm() {
        arm.setPower(0);
    }

    public static WobbleGoal standard(HardwareMap hardwareMap) {
        return new WobbleGoal(hardwareMap.servo.get("wobble"));
    }
}
