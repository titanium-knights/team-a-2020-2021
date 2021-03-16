package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private static final double RING_VEL = 60; // in in/s

    DcMotor compliance1;
    DcMotor compliance2;
    Servo pinball;
    Servo flap;
    int lastEncPos;

    private boolean isShooting;

    public Shooter(DcMotor compliance1, DcMotor compliance2, Servo pinball, Servo flap) {
        this.compliance1 = compliance1;
        this.compliance2 = compliance2;
        this.pinball = pinball;
        this.flap = flap;
        this.lastEncPos = compliance1.getCurrentPosition();

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
        compliance1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        compliance1.setPower(isShooting ? 9 : 0);
        compliance2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        compliance2.setPower(isShooting ? 9 : 0);
    }

    /**
     * Controls the pinball Servo that nudges the rings given two buttons in charge of moving it forward and backward respectively.
     * @param forwardButton The button on the controller mapped for moving the pinball forward.
     * @param backwardButton The button on the controller mapped for moving the pinball backward.
     */
    public void nudgeRings (boolean forwardButton, boolean backwardButton) {
        if (forwardButton && !backwardButton) {
            pinball.setPosition(0.15);
        } else {
            pinball.setPosition(0);
        }
    }
    public void raiseFlap() {
        flap.setPosition(0.05);
    }
    public void lowerFlap() {flap.setPosition(0.00);}

    // 1440 ticks per rotation, 2 in radius compliance wheels
    // 1440 ticks = 4*pi inches
    // speed in ticks/seconds -> inches/second

    public static Shooter standard(HardwareMap hardwareMap) {
        // HEY ANDREW, WHEN YOU CHANGE THE NAMES OF THE MOTORS, DO IT HERE THANK YOU :)))))) -anthonys
        DcMotor complianceLeft = hardwareMap.get(DcMotor.class, "shooter1");
        DcMotor complianceRight = hardwareMap.get(DcMotor.class, "shooter2");
        Servo pinball = hardwareMap.get(Servo.class, "pinball");
        Servo flap = hardwareMap.get(Servo.class, "flap");
        return new Shooter(complianceLeft, complianceRight, pinball, flap);
    }
}