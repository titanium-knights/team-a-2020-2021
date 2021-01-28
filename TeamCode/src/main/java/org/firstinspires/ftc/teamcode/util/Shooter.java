package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private static final double RING_VEL = 60; // in in/s

    DcMotor compliance;
    CRServo pinball;
    int lastEncPos;

    private boolean isShooting;

    public Shooter(DcMotor compliance, CRServo pinball) {
        this.compliance = compliance;
        this.pinball = pinball;
        this.lastEncPos = compliance.getCurrentPosition();

        SpeedAdjuster adjuster = new SpeedAdjuster();
        adjuster.start();
    }

    public class SpeedAdjuster extends Thread {

        public void run() {
            try {
                while (true) {
                    double currentVel = Math.abs(compliance.getCurrentPosition() - lastEncPos) / 0.5 * 0.008727; // in/sec
                    double targetPower = Math.min(1, Math.max(-1, RING_VEL / currentVel * compliance.getPower()));
                    compliance.setPower(targetPower);
                    lastEncPos = compliance.getCurrentPosition();
                    Thread.sleep(500);
                }
            } catch (Exception exc) { }
        }

    }

    /**
     * Toggles the power of the shooter's DcMotor.
     */
    public void toggleShooterPower () {
        isShooting = !isShooting;
        compliance.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        compliance.setPower(isShooting ? 9 : 0);
    }

    /**
     * Controls the pinball Servo that nudges the rings given two buttons in charge of moving it forward and backward respectively.
     * @param forwardButton The button on the controller mapped for moving the pinball forward.
     * @param backwardButton The button on the controller mapped for moving the pinball backward.
     */
    public void nudgeRings (boolean forwardButton, boolean backwardButton) {
        if (forwardButton && !backwardButton) {
            pinball.setPower(0.5f);
        } else if (!forwardButton && backwardButton) {
            pinball.setPower(-0.5f);
        } else {
            pinball.setPower(0);
        }
    }

    // 1440 ticks per rotation, 2 in radius compliance wheels
    // 1440 ticks = 4*pi inches
    // speed in ticks/seconds -> inches/second

    public static Shooter standard(HardwareMap hardwareMap) {
        DcMotor compliance = hardwareMap.get(DcMotor.class, "shooter");
        CRServo pinball = hardwareMap.get(CRServo.class, "pinball");
        pinball.setDirection(DcMotor.Direction.REVERSE);
        return new Shooter(compliance, pinball);
    }

}
