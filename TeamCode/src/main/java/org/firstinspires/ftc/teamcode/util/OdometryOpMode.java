package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OdometryOpMode extends LinearOpMode {

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
