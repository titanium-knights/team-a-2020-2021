package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Math.PI;

public abstract class OdometryOpMode extends LinearOpMode {

    public final double deadWheelCircumference = 1 * 2 * PI;
    public final double distanceVerticalEncoder = 1;
    public final double distanceStrafeEncoder = 1;

	public Vector2D position = null;
	public double angle = Double.NaN;

	@Override
	public void runOpMode() throws InterruptedException {

		odometryPreInit();

		if (angle == Double.NaN || position == null)
			throw new IllegalStateException("Initial position and angle should be set by odometryPreInit.");



		odometryPostInit();

		waitForStart();

		while (opModeIsActive()) {
			odometryLoop();
		}

		odometryClose();
	}

	public abstract void odometryPreInit();

	public abstract void odometryPostInit();

	public abstract void odometryLoop() throws InterruptedException;

	public abstract void odometryClose();

}
