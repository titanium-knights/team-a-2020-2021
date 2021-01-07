package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    DcMotor compliance;
    CRServo pinball;

    private boolean isShooting;

    public Shooter(DcMotor compliance, CRServo pinball) {
        this.compliance = compliance;
        this.pinball = pinball;
    }

    /**
     * Toggles the power of the shooter's DcMotor.
     */
    public void toggleShooterPower () {
        isShooting = !isShooting;
        compliance.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        compliance.setPower(isShooting ? 1 : 0);
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

    public static Shooter standard(HardwareMap hardwareMap) {
        DcMotor compliance = hardwareMap.get(DcMotor.class, "shooter");
        CRServo pinball = hardwareMap.get(CRServo.class, "pinball");
        pinball.setDirection(DcMotor.Direction.REVERSE);
        return new Shooter(compliance, pinball);
    }

}
