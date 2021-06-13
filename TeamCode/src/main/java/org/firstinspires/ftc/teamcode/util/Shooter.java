package org.firstinspires.ftc.teamcode.util;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Config
public class Shooter {

    private static final double RING_VEL = 60; // in in/s

    DcMotor shooter1;
    DcMotor shooter2;
    public Servo pinball;
    Servo flap;
    public static double kP = 0.6, kD = 0, fireSpeed = 22; //in between 18-20 m/s???
    public static double kV = 0.033, kS = 0, fireSpeedPS = 19;
    private PIDMath flyWheel;
    private FFFBMath flyWheelf;
    int lastEncPos;
    public static double rinterval = 25;
    private boolean retracted;
    public static double veloRange = 2;
    public double shots = 0;
    public static double leftPusherPos = 0;
    public static double rightPusherPos = .12;
    private ElapsedTime runtime = new ElapsedTime();


    ElapsedTime retractTime = new ElapsedTime();

    private boolean isShooting;
    private boolean isShootingPS;

    public Shooter(DcMotor shooter1, DcMotor shooter2, Servo pinball, Servo flap) {
        this.shooter1 = shooter1;
        this.shooter2 = shooter2;
        this.pinball = pinball;
        this.flap = flap;
        this.lastEncPos = shooter1.getCurrentPosition();
        flyWheel = new PIDMath(kP, 0, kD);
        flyWheelf = new FFFBMath(kV, 0, kS);

    }

    /**
     * Toggles the power of the shooter's DcMotor.
     */
    public void toggleShooterPower(double curVelo, double time) {
        isShooting = !isShooting;
        if (isShooting) {
            flyWheel.PIDConstants(kP, 0, kD);
            flyWheelf.FFConstants(kV, 0, kS);
            double gain = flyWheel.calculateGain(fireSpeed - curVelo, time) + flyWheelf.calculateFFFBGain(fireSpeed);
            shooter2.setPower(gain);
            shooter1.setPower(gain);
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }
    }
    public void shooterOn (double curVelo, double time) {
        isShooting = true;
        flyWheel.PIDConstants(kP, 0, kD);
        flyWheelf.FFConstants(kV, 0, kS);
        double gain = flyWheel.calculateGain(fireSpeed - curVelo, time) + flyWheelf.calculateFFFBGain(fireSpeed);
        shooter2.setPower(gain);
        shooter1.setPower(gain);
    }
        public void safetySwitch () {
        isShooting = false;
        shooter1.setPower(0);
        shooter2.setPower(0);
    }

    public boolean atSpeed(double curVelo) {
        if (Math.abs(curVelo - fireSpeed) < veloRange)
            return true;
        return false;
    }
    public boolean atSpeedPS(double curVelo) {
        if (Math.abs(curVelo - fireSpeedPS) < veloRange)
            return true;
        return false;
    }

    public void timedFireN(double curVelo) {
        if (retracted) {
            if (shots == 2 && atSpeed(curVelo)) {
                timedShot();
            } else if (shots == 1 && atSpeed(curVelo)) {
                timedShot();
            } else if (atSpeed(curVelo) && shots == 0) {
                timedShot();
            }
            if (shots == 3) {
                timedCancel();
            }
        } else {
            if (retractTime.milliseconds() > 2 * rinterval) {
                pinball.setPosition(leftPusherPos);
                retracted = true;
            }
        }
    }

    public void timedShot() {
        pinball.setPosition(rightPusherPos);
        shots++;
        retractTime.reset();
        retracted = false;
    }

    public void timedCancel() {
        shots = 0;
        retractTime.reset();
        pinball.setPosition(leftPusherPos);
    }

        public void nudgeRings (boolean forwardButton, boolean backwardButton) {
            if (forwardButton && !backwardButton) {
                pinball.setPosition(0.15);
            } else {
                pinball.setPosition(0);
            }
        }
        public void raiseFlap () {
            flap.setPosition(0.05);
        }
        public void lowerFlap () {
            flap.setPosition(0.00);
        }

        public static Shooter standard (HardwareMap hardwareMap){
            DcMotor shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
            DcMotor shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
            Servo pinball = hardwareMap.get(Servo.class, "pinball");
            Servo flap = hardwareMap.get(Servo.class, "Flap");
            return new Shooter(shooter1, shooter2, pinball, flap);
        }
        }

