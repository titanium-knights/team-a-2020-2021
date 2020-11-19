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

	/**
	 * Initialization of variables before odometry initialization; must set initial {@code position} and {@code angle}
	 */
	public abstract void odometryPreInit();

	/**
	 * Initialization of variables after odometry initialization
	 */
	public abstract void odometryPostInit();

	/**
	 * Actual opmode process which is run once every tick loop; odometry gets updated before this is called
	 * @throws InterruptedException
	 */
	public abstract void odometryLoop() throws InterruptedException;

	/**
	 * Ending process after close of tick loop
	 */
	public abstract void odometryClose();

}
