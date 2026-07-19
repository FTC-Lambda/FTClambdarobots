package org.firstinspires.ftc.teamcode.vision;

/**
 * Tunable thresholds for DECODE Limelight ball detection, grouping, scoring,
 * and target persistence. Area is an apparent-size heuristic (percent of image),
 * not true physical distance.
 */
public final class BallVisionConfig {

	private BallVisionConfig() {}

	public static final int PIPELINE_APRILTAG = 0;
	public static final int PIPELINE_BALL = 1;
	public static final int POLL_RATE_HZ = 50;

	// --- Detection filtering ---
	/** Minimum detector confidence (SDK returns 0–100 style scores in practice). */
	public static final double MIN_CONFIDENCE = 40.0;
	/** Minimum target area as percent of image (0–100). */
	public static final double MIN_TARGET_AREA = 0.15;
	/** Reject Limelight frames older than this (ms), from {@code LLResult.getStaleness()}. */
	public static final long MAX_RESULT_STALENESS_MS = 250;
	/** Reject detections whose center is within this fraction of the image edge (0–0.5). */
	public static final double EDGE_MARGIN_FRACTION = 0.05;
	/** Approximate Limelight 3A stream width/height used for edge checks (pixels). */
	public static final double IMAGE_WIDTH_PX = 320.0;
	public static final double IMAGE_HEIGHT_PX = 240.0;

	// --- Grouping proximity (prefer bounding-box gaps; tx/ty is fallback) ---
	/** Max horizontal gap between boxes, as a multiple of average box width. */
	public static final double MAX_HORIZONTAL_GAP_IN_WIDTHS = 0.75;
	/** Max vertical gap between boxes, as a multiple of average box height. */
	public static final double MAX_VERTICAL_GAP_IN_HEIGHTS = 0.75;
	/** Angular fallback when corners/pixels are unavailable (degrees). */
	public static final double MAX_TX_DIFF_DEG = 8.0;
	public static final double MAX_TY_DIFF_DEG = 8.0;

	// --- Best-group scoring weights (higher = more influence) ---
	public static final double SCORE_CENTERLINE = 2.0;
	public static final double SCORE_SIZE = 1.5;
	public static final double SCORE_AREA = 1.0;
	public static final double SCORE_CONFIDENCE = 1.0;
	public static final double SCORE_DESIRED_COLOR = 2.5;
	public static final double PENALTY_EDGE = 1.5;
	public static final double PENALTY_WRONG_COLOR = 3.0;
	public static final double PENALTY_LOW_CONFIDENCE = 1.0;
	public static final double CENTERLINE_SOFT_DEG = 20.0;

	// --- Target persistence ---
	/** Keep last valid target this long after vision loss (ms). */
	public static final long VISION_LOSS_TIMEOUT_MS = 350;
	/** Associate a new group with the previous target when centers are within this (deg). */
	public static final double PERSIST_ASSOCIATION_DEG = 12.0;
}
