package org.firstinspires.ftc.teamcode.vision.hive;

import java.util.Locale;

/** Immutable 3D point/vector in inches. */
public final class Vec3 {
	public final double x;
	public final double y;
	public final double z;

	public Vec3(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	@Override
	public String toString() {
		return String.format(Locale.US, "(%.2f, %.2f, %.2f)", x, y, z);
	}
}
