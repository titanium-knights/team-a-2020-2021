package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class WobbleGoal {
    public Servo grabber;
    public Servo grabber2;
    public DcMotor arm;

    public WobbleGoal(Servo grabber, Servo grabber2, DcMotor arm) {
        this.grabber = grabber;
        this.arm = arm;
        this.grabber2 = grabber2;

        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void release() {
        grabber.setPosition(0.5);
        grabber2.setPosition(0.5);
    }
    public void grab() {
        grabber.setPosition(0.05);
        grabber2.setPostition(0.05);
    }
    public void stopGrabber() {}

    public void liftArm() {
        this.arm1.setPosition(0);
        this.arm2.setPosition(1);
    }
    public void lowerArm() {
            this.arm1.setPosition(1);
            this.arm2.setPosition(0);
        }
    public void stopArm() {}

/*
    // old motor arm stuff
    public void liftArm() {
        arm.setPower(0.6);
    }
    public void lowerArm() {

    }
    public void stopArm() {
        arm.setPower(0);
    } */

    public static WobbleGoal standard(HardwareMap hardwareMap) {
        //return new WobbleGoal(hardwareMap.servo.get("wobble"), hardwareMap.dcMotor.get("wobble_arm"));
        return new WobbleGoal(hardwareMap.servo.get("wobble"), hardwareMap.servo.get("arm1" ), hardwareMap.servo.get("arm2"));

    }

    public void wobbleGrabber() {
        arm.setPower(-0.6);
        grabber.setPosition(0.05);
        grabber2.setPosition(0.05);
        arm.setPower(0.6);
    }

    public void wobbleRelease() {
        arm.setPower(-0.6);
        grabber.setPosition(0.5);
        grabber2.setPosition(0.5);
        arm.setPower(0.6);
    }
}
