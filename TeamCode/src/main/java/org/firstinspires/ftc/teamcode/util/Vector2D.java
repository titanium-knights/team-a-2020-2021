package org.firstinspires.ftc.teamcode.util;

import java.util.Vector;

public class Vector2D {
	public final double x;
	public final double y;
	public final double r;
	public final double t;

	public Vector2D(double x, double y, double r, double t) {
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

	public Vector2D rotatedBy(double theta) {
		return Vector2D.polar(r, t + theta);
	}

}