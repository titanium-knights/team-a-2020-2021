package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private static final double RING_VEL = 60; // in in/s

    DcMotor complianceLeft;
    DcMotor complianceRight;
    CRServo pinball;
    int lastEncPos;

    private boolean isShooting;

    public Shooter(DcMotor complianceLeft, DcMotor complianceRight, CRServo pinball) {
        this.complianceLeft = complianceLeft;
        this.complianceRight = complianceRight;
        this.pinball = pinball;
        this.lastEncPos = complianceLeft.getCurrentPosition();

        //SpeedAdjuster adjuster = new SpeedAdjuster();
        //adjuster.start();
    }
    /*
    public class SpeedAdjuster extends Thread {

        public void run() {
            try {
                while (true) {
                    double currentVel = Math.abs(complianceLeft.getCurrentPosition() - lastEncPos) / 0.5 * 0.008727; // in/sec
                    double targetPower = Math.min(1, Math.max(-1, RING_VEL / currentVel * complianceLeft.getPower()));
                    complianceLeft.setPower(targetPower);
                    complianceRight.setPower(targetPower);
                    lastEncPos = complianceLeft.getCurrentPosition();
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
        complianceLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        complianceLeft.setPower(isShooting ? 9 : 0);
        complianceRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        complianceRight.setPower(isShooting ? 9 : 0);
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
        // HEY ANDREW, WHEN YOU CHANGE THE NAMES OF THE MOTORS, DO IT HERE THANK YOU :)))))) -anthonys
        DcMotor complianceLeft = hardwareMap.get(DcMotor.class, "shooter1");
        DcMotor complianceRight = hardwareMap.get(DcMotor.class, "shooter2");
        CRServo pinball = hardwareMap.get(CRServo.class, "pinball");
        pinball.setDirection(DcMotor.Direction.REVERSE);
        return new Shooter(complianceLeft, complianceRight, pinball);
    }

}
