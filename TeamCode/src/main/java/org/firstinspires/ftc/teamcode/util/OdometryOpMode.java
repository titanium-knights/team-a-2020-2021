package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OdometryOpMode extends LinearOpMode {

    public final double deadWheelCircumference = 1*2*Math.PI;
    public final double distanceVerticalEncoder = 1;
    public final double distanceStrafeEncoder = 1;

	public Vector2D position;
	public double angle;

	@Override
	public void runOpMode() throws InterruptedException {

		// Starting position; change later
		position = Vector2D.cartesian(0, 0);
		angle = Math.PI / 2;

		odometryStart();
		waitForStart();

		while (opModeIsActive()) {
			odometryLoop();
		}

		odometryClose();
	}

	public abstract void odometryStart();

	public abstract void odometryLoop() throws InterruptedException;

	public abstract void odometryClose();

}
