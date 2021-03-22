package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public static final int LOWERED_ENCODER_POS = -5500;

public class WobbleGoal {
    public Servo grabber;
    public CRServo arm1;
    public CRServo arm2;
    public Encoder encoder;
    public int initialPosition;

    public WobbleGoal(Servo grabber, CRServo arm1, CRServo arm2, Encoder encoder) {
        this.grabber = grabber;
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.encoder = encoder;
        this.initialPosition = encoder.getCurrentPosition();

        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void release() {
        grabber.setPosition(0.5);
    }
    public void grab() {
        grabber.setPosition(0.05);
    }
    public void stopGrabber() {}

    public void liftArm() {
        arm1.setPower(-1);
        arm2.setPower(1);
    }
    public void lowerArm() {
            arm1.setPower(1);
            arm2.setPower(-1);
        }
    public void stopArm() {
        arm1.setPower(0);
        arm2.setPower(0);
    }

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
        return new WobbleGoal(hardwareMap.servo.get("wobble"), hardwareMap.crservo.get("arm1" ), hardwareMap.crservo.get("arm2"), new Encoder(hardwareMap.get(DcMotorEx.class, "intake_top")));
    }

    public void wobbleGrabber() {
        lowerArm();
        grabber.setPosition(0.05);
        liftArm();
    }

    public void wobbleRelease() {
        lowerArm();
        grabber.setPosition(0.5);
        liftArm();
    }

    public void raiseToFoldedPos(LinearOpMode opMode) {
        liftArm();
        while (getPosition() < 0) {
            opMode.sleep(10);
        }
        stopArm();
    }

    public void lowerToLoweredPos(LinearOpMode opMode) {
        lowerArm();
        while (getPosition() > LOWERED_ENCODER_POS) {
            opMode.sleep(10);
        }
        stopArm();
    }

    public int getPosition() {
        return encoder.getCurrentPosition() - initialPosition;
    }
}
