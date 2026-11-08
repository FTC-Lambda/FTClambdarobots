package org.firstinspires.ftc.teamcode.vision.hive;

import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Robot field pose from the HIVE AprilTags (both alliances' HIVEs). Pure Java.
 *
 * <p>Per frame:</p>
 * <ol>
 *   <li>Each sighting becomes a tag position in the robot frame ({@link CameraMount}).</li>
 *   <li>Each cluster's median measured height says whether that CELL is RAISED or LOWERED
 *       (same thresholds as HIVE state detection). Ambiguous clusters are not used. This
 *       tells us where the moving tags are without needing the HIVE state from elsewhere.</li>
 *   <li>{@link HiveFieldModel} gives each tag's field position for that CELL position.</li>
 *   <li>With a known heading (gyro), every tag gives an x/y fix and the fixes are averaged.
 *       Without one, x, y and heading are fitted together from 2+ tags that are at least
 *       {@code minBaselineIn} apart (2D least-squares rigid fit).</li>
 *   <li>Tags disagreeing with the fit by more than {@code maxResidualIn} are dropped once
 *       and the fit is repeated.</li>
 * </ol>
 *
 * <p>Only tag positions are used, never the Limelight's per-tag rotation estimate, which is
 * noisy on 3.25 in tags at shooting range.</p>
 */
public final class HiveLocalizer {

	private final HiveFieldModel model;
	private CameraMount mount;
	private HiveDetectionConfig classification;
	private final double maxResidualIn;
	private final double minBaselineIn;

	public HiveLocalizer(HiveFieldModel model, CameraMount mount, HiveDetectionConfig classification,
			double maxResidualIn, double minBaselineIn) {
		if (model == null || mount == null || classification == null) {
			throw new IllegalArgumentException("model, mount and classification are required");
		}
		this.model = model;
		this.mount = mount;
		this.classification = classification;
		this.maxResidualIn = maxResidualIn;
		this.minBaselineIn = minBaselineIn;
	}

	/** Default model, mount from Constants, default classification, 6 in residual gate, 4 in baseline. */
	public static HiveLocalizer defaults() {
		return new HiveLocalizer(HiveFieldModel.defaults(), CameraMount.fromConstants(),
				HiveDetectionConfig.defaults(), 6.0, 4.0);
	}

	public void setMount(CameraMount mount) {
		if (mount != null) {
			this.mount = mount;
		}
	}

	public CameraMount getMount() {
		return mount;
	}

	public void setClassification(HiveDetectionConfig classification) {
		if (classification != null) {
			this.classification = classification;
		}
	}

	public HiveFieldModel getModel() {
		return model;
	}

	/** A tag the localizer will use: where the robot sees it and where the model says it is. */
	private static final class Match {
		final int id;
		final Vec3 robot;
		final Vec3 field;

		Match(int id, Vec3 robot, Vec3 field) {
			this.id = id;
			this.robot = robot;
			this.field = field;
		}
	}

	/**
	 * @param sightings  tags from one camera frame (non-HIVE ids are ignored)
	 * @param headingDeg robot field heading in degrees if known (gyro), or NaN to fit it from tags
	 */
	public FieldPoseEstimate estimate(List<TagSighting> sightings, double headingDeg) {
		// 1. Robot-frame positions, grouped by cluster (index = (id - 30) / 4).
		List<List<Integer>> ids = new ArrayList<>();
		List<List<Vec3>> pts = new ArrayList<>();
		for (int i = 0; i < 4; i++) {
			ids.add(new ArrayList<Integer>());
			pts.add(new ArrayList<Vec3>());
		}
		if (sightings != null) {
			for (TagSighting s : sightings) {
				if (s == null || !HiveTagMap.isHiveTag(s.id)) {
					continue;
				}
				Vec3 p = mount.toRobotFrame(s.rangeIn, s.txDeg, s.tyDeg);
				if (p == null || p.z < classification.minPlausibleHeightIn
						|| p.z > classification.maxPlausibleHeightIn) {
					continue;
				}
				int c = (s.id - 30) / 4;
				ids.get(c).add(s.id);
				pts.get(c).add(p);
			}
		}

		// 2-3. Classify each cluster by median height and look up modeled tag positions.
		List<Match> matches = new ArrayList<>();
		List<String> clusters = new ArrayList<>();
		for (int c = 0; c < 4; c++) {
			List<Vec3> cp = pts.get(c);
			if (cp.isEmpty()) {
				continue;
			}
			double[] z = new double[cp.size()];
			for (int i = 0; i < z.length; i++) {
				z[i] = cp.get(i).z;
			}
			CellPosition pos = classify(median(z));
			int firstId = 30 + 4 * c;
			Alliance alliance = HiveTagMap.allianceOf(firstId);
			HiveCell cell = HiveTagMap.cellOf(firstId);
			clusters.add(HiveLabels.shortCode(alliance, cell) + " " + pos);
			if (pos != CellPosition.RAISED && pos != CellPosition.LOWERED) {
				continue;
			}
			for (int i = 0; i < cp.size(); i++) {
				int id = ids.get(c).get(i);
				matches.add(new Match(id, cp.get(i), model.tagPosition(id, pos)));
			}
		}
		if (matches.isEmpty()) {
			return FieldPoseEstimate.none(clusters.isEmpty() ? "no HIVE tags" : "clusters ambiguous",
					clusters, 0);
		}

		// 4-5. Fit, reject outliers once, refit.
		boolean headingKnown = !Double.isNaN(headingDeg) && !Double.isInfinite(headingDeg);
		FieldPoseEstimate first = fit(matches, headingDeg, headingKnown, clusters, 0);
		if (!first.valid) {
			return first;
		}
		List<Match> kept = new ArrayList<>();
		for (Match m : matches) {
			if (residual(m, first.xIn, first.yIn, first.headingDeg) <= maxResidualIn) {
				kept.add(m);
			}
		}
		int rejected = matches.size() - kept.size();
		if (rejected == 0) {
			return first;
		}
		if (kept.isEmpty()) {
			return FieldPoseEstimate.none("tags disagree", clusters, rejected);
		}
		return fit(kept, headingDeg, headingKnown, clusters, rejected);
	}

	private FieldPoseEstimate fit(List<Match> ms, double headingDeg, boolean headingKnown,
			List<String> clusters, int rejected) {
		double h;
		double x;
		double y;
		FieldPoseEstimate.Mode mode;
		if (headingKnown) {
			h = Math.toRadians(headingDeg);
			double cos = Math.cos(h);
			double sin = Math.sin(h);
			double sx = 0.0;
			double sy = 0.0;
			for (Match m : ms) {
				sx += m.field.x - (cos * m.robot.x - sin * m.robot.y);
				sy += m.field.y - (sin * m.robot.x + cos * m.robot.y);
			}
			x = sx / ms.size();
			y = sy / ms.size();
			mode = FieldPoseEstimate.Mode.HEADING_GIVEN;
		} else {
			if (ms.size() < 2 || maxSpread(ms) < minBaselineIn) {
				return FieldPoseEstimate.none(ms.size() < 2 ? "need 2+ tags (no heading)"
						: "tags too close together (no heading)", clusters, rejected);
			}
			double crx = 0.0;
			double cry = 0.0;
			double cfx = 0.0;
			double cfy = 0.0;
			for (Match m : ms) {
				crx += m.robot.x;
				cry += m.robot.y;
				cfx += m.field.x;
				cfy += m.field.y;
			}
			int n = ms.size();
			crx /= n;
			cry /= n;
			cfx /= n;
			cfy /= n;
			double sCos = 0.0;
			double sSin = 0.0;
			for (Match m : ms) {
				double ax = m.robot.x - crx;
				double ay = m.robot.y - cry;
				double bx = m.field.x - cfx;
				double by = m.field.y - cfy;
				sCos += ax * bx + ay * by;
				sSin += ax * by - ay * bx;
			}
			h = Math.atan2(sSin, sCos);
			x = cfx - (Math.cos(h) * crx - Math.sin(h) * cry);
			y = cfy - (Math.sin(h) * crx + Math.cos(h) * cry);
			mode = FieldPoseEstimate.Mode.TAGS_ONLY;
		}
		double hDeg = Math.toDegrees(h);
		double sumSq = 0.0;
		double sumSqZ = 0.0;
		for (Match m : ms) {
			double r = residual(m, x, y, hDeg);
			sumSq += r * r;
			double dz = m.robot.z - m.field.z;
			sumSqZ += dz * dz;
		}
		return new FieldPoseEstimate(true, mode, x, y, normalizeDeg(hDeg), ms.size(), rejected,
				clusters, Math.sqrt(sumSq / ms.size()), Math.sqrt(sumSqZ / ms.size()), "ok");
	}

	/** Horizontal distance between a tag's modeled field position and where the pose puts it. */
	private static double residual(Match m, double x, double y, double headingDeg) {
		double h = Math.toRadians(headingDeg);
		double px = x + Math.cos(h) * m.robot.x - Math.sin(h) * m.robot.y;
		double py = y + Math.sin(h) * m.robot.x + Math.cos(h) * m.robot.y;
		return Math.hypot(px - m.field.x, py - m.field.y);
	}

	private static double maxSpread(List<Match> ms) {
		double best = 0.0;
		for (int i = 0; i < ms.size(); i++) {
			for (int j = i + 1; j < ms.size(); j++) {
				best = Math.max(best, Math.hypot(ms.get(i).field.x - ms.get(j).field.x,
						ms.get(i).field.y - ms.get(j).field.y));
			}
		}
		return best;
	}

	private CellPosition classify(double heightIn) {
		if (heightIn >= classification.raisedThresholdIn + classification.hysteresisIn) {
			return CellPosition.RAISED;
		}
		if (heightIn <= classification.raisedThresholdIn - classification.hysteresisIn) {
			return CellPosition.LOWERED;
		}
		return CellPosition.AMBIGUOUS;
	}

	private static double median(double[] v) {
		double[] s = v.clone();
		Arrays.sort(s);
		int n = s.length;
		return n % 2 == 1 ? s[n / 2] : (s[n / 2 - 1] + s[n / 2]) / 2.0;
	}

	static double normalizeDeg(double deg) {
		double d = deg % 360.0;
		if (d > 180.0) {
			d -= 360.0;
		} else if (d <= -180.0) {
			d += 360.0;
		}
		return d;
	}
}
