package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OdometryOpMode extends LinearOpMode {
    public final double deadWheelDiameter = 1*2*Math.PI;
    public final double distanceVerticalEncoder = 1;
    public final double distanceStrafeEncoder = 1;

	public Vector2D position;
	public Vector2D angle;

	@Override
	public void runOpMode() throws InterruptedException {

		odometryStart();
		waitForStart();

		while (opModeIsActive()) {
			odometryLoop();
		}

		odometryClose();
	}

	public abstract void odometryStart();

	public abstract void odometryLoop();

	public abstract void odometryClose();

}
