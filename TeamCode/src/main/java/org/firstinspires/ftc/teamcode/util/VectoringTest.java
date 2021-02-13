package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VectoringTest {
    public CRServo Right;
    public CRServo Left;

    public VectoringTest(CRServo Right, CRServo Left) {
        this.Right = Right;
        this.Left = Left;
    }

    public void RunVectorsWithPower(CRServo Right, CRServo Left){
        this.Left.setPower(1);
        this.Right.setPower(-1);
    }

    public void StopVectors(CRServo Right, CRServo Left) {
        this.Left.setPower(0);
        this.Right.setPower(0);
    }

    public void ReverseVectorsWithPower(CRServo Right, CRServo Left) {
        this.Left.setPower(-1);
        this.Right.setPower(1);
    }

    public static VectoringTest standard(HardwareMap hardwareMap) {
        return new VectoringTest(hardwareMap.crservo.get("Left_Vector"), hardwareMap.crservo.get("Right_Vector"));
    }

    }
