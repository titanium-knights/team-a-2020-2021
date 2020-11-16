package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OdometryOpMode extends LinearOpMode {
    public final double deadWheelDiameter = 1*2*Math.PI;
    public final double distanceVerticalEncoder = 1;
    public final double distanceStrafeEncoder = 1;

	@Override
	public void runOpMode() throws InterruptedException {

		open();
		waitForStart();

		while (opModeIsActive()) {
			loop();
		}

		close();
	}

	public abstract void open();

	public abstract void loop(Vector2D position, Vector2D angle);

	public abstract void close();

}
