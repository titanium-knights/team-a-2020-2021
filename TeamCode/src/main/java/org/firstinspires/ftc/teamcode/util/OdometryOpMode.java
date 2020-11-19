package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Math.PI;

public abstract class OdometryOpMode extends LinearOpMode {

    public final double deadWheelCircumference = 1 * 2 * PI;
    public final double distanceVerticalEncoder = 1;
    public final double distanceStrafeEncoder = 1;

	public Vector2D position;
	public double angle;

	@Override
	public void runOpMode() throws InterruptedException {

		// Starting position; change later
		position = getStartingPosition();
		angle = getStartingAngle();

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

	/**
	 * To be hard-coded per opmode; returns the initial position of the robot.
	 */
	public abstract Vector2D getStartingPosition();

	/**
	 * To be hard-coded per opmode; returns the initial angle of the robot from positive x. Default is PI/2.
	 */
	public double getStartingAngle() {
		return PI / 2;
	}

}
