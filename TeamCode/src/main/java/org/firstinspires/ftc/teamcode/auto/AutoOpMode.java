package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.*;

@Autonomous(name = "Auto Op Mode")
public class AutoOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // make sure to angle robot left - curve will be fixed at a later date
        // TODO for hardware team

        double MS_PER_INCHES = 1000/54.0;
        double POWER = 0.7;

        MecanumDrive drive = MecanumDrive.standard(hardwareMap);
        for (DcMotor motor: drive.getMotors()) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        DcMotor shooter = hardwareMap.dcMotor.get("shooter");
        CRServo shooterServo = hardwareMap.crservo.get("pinball");

        waitForStart();

        // Movey move
        shooter.setPower(-1);
        drive.forwardWithPower(POWER);
        sleep((long)(12 * MS_PER_INCHES));
        drive.stop();

        // Shooty shoot
        for (int i = 0; i < 3; ++i) {
            // Thwack thwack thwack
            sleep(1000);
            shooterServo.setPower(0.5);
            sleep(1000);
            shooterServo.setPower(-0.5);
            sleep(500);
            shooterServo.setPower(0);
            sleep(1000);
        }

        shooter.setPower(0);

        // Parky park
        drive.forwardWithPower(POWER);
        sleep(350);
        drive.stop();
    }
}