package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;

@TeleOp(name = "Op Test E1", group = "Erez")
public class OpTestE1 extends LinearOpMode {

	private static final double JOYSTICK_SENSITIVITY = 0.2f;
	private static final long MSPI = 1000;
	private static final long MSPT = (long) (MSPI / 12.0 + 0.5);
	private static final boolean QUANTIZE = true;
	private static final byte MIRROR = 1;

	public static String[] standardMotorNames = {"mecanum_fl", "mecanum_fr", "mecanum_bl", "mecanum_br"};
	public static Vector2D[] standardMotorVectors = {Vector2D.cartesian(-1, 1), Vector2D.cartesian(-1, -1), Vector2D.cartesian(1, 1), Vector2D.cartesian(1, -1)};
	static DcMotor.Direction[] standardMotorDirections = {DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE};

	private DcMotor[] motors = new DcMotor[4];

	@Override
	public void runOpMode() throws InterruptedException {

		for (int i = 0; i < motors.length; i++)
			motors[i] = hardwareMap.get(DcMotor.class, standardMotorNames[i]);

		Vector2D[] dirVectors = new Vector2D[4];

		waitForStart();

		while (opModeIsActive()) {

			dirVectors[0] = dirVectors[2] = leftVector();
			dirVectors[1] = dirVectors[3] = rightVector();

			for (int i = 0; i < motors.length; i++)
				motors[i].setPower(dirVectors[i].dot(standardMotorVectors[i]));

		}
	}

	public static double joystickQuantize(double toQuantize) {
		return QUANTIZE && Math.abs(toQuantize) < JOYSTICK_SENSITIVITY ? 0 : toQuantize;
	}

	public Vector2D leftVector() {
		return Vector2D.cartesian(MIRROR * joystickQuantize(gamepad1.left_stick_x), joystickQuantize(gamepad1.left_stick_y));
	}

	public Vector2D rightVector() {
		return Vector2D.cartesian(MIRROR * joystickQuantize(gamepad1.right_stick_x), joystickQuantize(gamepad1.right_stick_y));
	}

	private static class Vector2D {
		public final double x;
		public final double y;
		public final double r;
		public final double t;

		private Vector2D(double x, double y, double r, double t) {
			this.x = x;
			this.y = y;
			this.r = r;
			this.t = t;
		}

		public static Vector2D cartesian(double x, double y) {
			return new Vector2D(x, y, Math.sqrt(x*x + y*y), Math.atan2(y, x));
		}

		public static Vector2D polar(double r, double t) {
			return new Vector2D(r * Math.cos(t), r * Math.sin(t), r, t);
		}

		public double dot(Vector2D other) {
			return x * other.x + y * other.y;
		}

		public double cross(Vector2D other) {
			return x * other.y - y * other.x;
		}

		public Vector2D plus(Vector2D other) {
			return cartesian(x + other.x, y + other.y);
		}

		public Vector2D times(double scalar) {
			return Vector2D.cartesian(x * scalar, y * scalar);
		}

		public Vector2D negative() {
			return times(-1);
		}

	}

}