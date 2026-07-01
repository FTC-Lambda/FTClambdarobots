package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.json.JSONObject;

import java.lang.reflect.Method;

/**
 * Helpers for configuring Limelight pipelines from robot code, so a Limelight can be
 * set up without the web UI (which a laptop on the robot WiFi usually can't reach — the
 * Control Hub is on the Limelight's USB-Ethernet subnet, the laptop is not).
 *
 * WHY REFLECTION: the FTC SDK's Limelight3A intentionally makes pipeline read/write
 * private (getPipelineAtIndex / updatePipeline); only getStatus/pipelineSwitch/etc. are
 * public. Those private methods already do the right thing — talk to the Limelight REST
 * API (port 5807) using the device's own IP and the SDK's request/threading logic — so we
 * call them by reflection rather than re-implementing raw HTTP and IP parsing ourselves.
 * Limelight3A is an app-bundled class (not an android.* framework class), so setAccessible
 * is unrestricted here.
 *
 * All edits are read-modify-write, so unrelated pipeline settings (crop, exposure,
 * calibration, ...) are preserved.
 */
public final class LimelightPipelines {

	/** AprilTag family used by FTC. */
	public static final String FTC_FIDUCIAL_FAMILY = "aprilClassic36h11";

	// Cached reflected handles to the private SDK methods. Resolved once, lazily.
	private static Method getPipelineAtIndex; // JSONObject getPipelineAtIndex(int)
	private static Method updatePipeline;     // boolean updatePipeline(JSONObject, boolean)
	private static boolean reflectionResolved;

	private LimelightPipelines() {}

	private static synchronized boolean resolveReflection() {
		if (reflectionResolved) {
			return getPipelineAtIndex != null && updatePipeline != null;
		}
		reflectionResolved = true;
		try {
			getPipelineAtIndex = Limelight3A.class.getDeclaredMethod("getPipelineAtIndex", int.class);
			getPipelineAtIndex.setAccessible(true);
			updatePipeline = Limelight3A.class.getDeclaredMethod(
					"updatePipeline", JSONObject.class, boolean.class);
			updatePipeline.setAccessible(true);
			return true;
		} catch (Throwable t) {
			// SDK changed these method names/signatures — fail gracefully.
			getPipelineAtIndex = null;
			updatePipeline = null;
			return false;
		}
	}

	/** Downloads a slot's pipeline JSON, or null if unreachable / reflection unavailable. */
	private static JSONObject readPipeline(Limelight3A limelight, int index) {
		if (!resolveReflection()) return null;
		try {
			return (JSONObject) getPipelineAtIndex.invoke(limelight, index);
		} catch (Throwable t) {
			return null;
		}
	}

	/** Uploads pipeline JSON to the active slot; flush=true persists it to disk. */
	private static boolean writePipeline(Limelight3A limelight, JSONObject pipe, boolean flush) {
		if (!resolveReflection()) return false;
		try {
			Object result = updatePipeline.invoke(limelight, pipe, flush);
			return Boolean.TRUE.equals(result);
		} catch (Throwable t) {
			return false;
		}
	}

	/** Returns the pipeline_type field of the given slot, or null if it can't be read. */
	public static String pipelineType(Limelight3A limelight, int index) {
		JSONObject pipe = readPipeline(limelight, index);
		return pipe != null ? pipe.optString("pipeline_type", null) : null;
	}

	/** True if the given slot is currently a fiducial (AprilTag) pipeline. */
	public static boolean isFiducial(Limelight3A limelight, int index) {
		return "pipe_fiducial".equals(pipelineType(limelight, index));
	}

	/**
	 * Ensures the given slot is an AprilTag pipeline with a full-frame crop, converting/
	 * fixing it in place if needed. No-op (and returns true) if it's already a fiducial
	 * pipeline AND already uses the full sensor frame.
	 *
	 * The crop reset matters: a narrowed crop (e.g. inherited from a color pipeline this was
	 * converted from) shrinks the usable field of view — especially top/bottom — so tags
	 * high or low drop out. Resetting crop to +/-1 opens the frame back up to the full lens.
	 *
	 * NOTE: updatePipeline writes to the ACTIVE pipeline, so the caller should have already
	 * called limelight.pipelineSwitch(index) before this.
	 *
	 * @param sizeMm printed tag edge length in millimeters (affects 3D pose accuracy only)
	 * @return true if the slot is fiducial after this call, false if the read/write failed
	 */
	public static boolean ensureFiducial(Limelight3A limelight, int index,
										 String family, double sizeMm) {
		JSONObject pipe = readPipeline(limelight, index);
		if (pipe == null) return false;                      // device unreachable / no reflection
		boolean isFiducial = "pipe_fiducial".equals(pipe.optString("pipeline_type", null));
		boolean threeDEnabled = pipe.optInt("fiducial_skip3d", 0) == 0;
		if (isFiducial && isFullFrame(pipe) && threeDEnabled) {
			return true;                                     // already correct, nothing to write
		}
		try {
			pipe.put("pipeline_type", "pipe_fiducial");
			pipe.put("fiducial_type", family);
			pipe.put("fiducial_size", sizeMm);
			// Open the crop window to the full sensor so nothing high/low is cut off.
			pipe.put("crop_x_min", -1.0);
			pipe.put("crop_x_max", 1.0);
			pipe.put("crop_y_min", -1.0);
			pipe.put("crop_y_max", 1.0);
			// Enable the 3D pose solve (0 = don't skip). Without this, tx/ty work but all
			// pose/distance fields come back zeroed.
			pipe.put("fiducial_skip3d", 0);
		} catch (Exception e) {
			return false;
		}
		return writePipeline(limelight, pipe, true);
	}

	/** True if the pipeline's crop window is (near) the full sensor frame, i.e. +/-1 on both axes. */
	private static boolean isFullFrame(JSONObject pipe) {
		return pipe.optDouble("crop_x_min", -1.0) <= -0.999
			&& pipe.optDouble("crop_x_max", 1.0) >= 0.999
			&& pipe.optDouble("crop_y_min", -1.0) <= -0.999
			&& pipe.optDouble("crop_y_max", 1.0) >= 0.999;
	}
}
