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
	// SCALE NOTE: the Limelight reports detector confidence and target area as 0–1
	// fractions, even though the SDK javadoc for DetectorResult claims 0–100. Measured
	// on our hardware, good ball detections report conf 0.7–0.8 and ta 0.01–0.02.
	// Every threshold and score normalization below uses that 0–1 fraction scale.
	/** Minimum detector confidence, as a 0–1 fraction. */
	public static final double MIN_CONFIDENCE = 0.50;
	/** Minimum target area, as a 0–1 fraction of the image. */
	public static final double MIN_TARGET_AREA = 0.004;
	/** Reject Limelight frames older than this (ms), from {@code LLResult.getStaleness()}. */
	public static final long MAX_RESULT_STALENESS_MS = 250;

	// --- Edge rejection (angular, so it never depends on stream resolution) ---
	/** Reject detections beyond this horizontal angle; an LL3A sees roughly ±27°. */
	public static final double EDGE_MAX_TX_DEG = 24.0;
	/** Reject detections above this angle; floor balls are never high in the frame. */
	public static final double EDGE_MAX_TY_DEG = 18.0;
	/**
	 * Reject detections below this angle. Deliberately generous, because a ball close to
	 * the intake sits low in the frame and clipping it here would blind the robot exactly
	 * when it is about to collect.
	 */
	public static final double EDGE_MIN_TY_DEG = -26.0;
	/** Groups within this margin of the edge limits are penalized rather than rejected. */
	public static final double EDGE_PENALTY_MARGIN_DEG = 4.0;

	// --- Grouping proximity (prefer bounding-box gaps; tx/ty is fallback) ---
	/** Max horizontal gap between boxes, as a multiple of average box width. */
	public static final double MAX_HORIZONTAL_GAP_IN_WIDTHS = 0.75;
	/** Max vertical gap between boxes, as a multiple of average box height. */
	public static final double MAX_VERTICAL_GAP_IN_HEIGHTS = 0.75;
	/** Angular fallback when corners/pixels are unavailable (degrees). */
	public static final double MAX_TX_DIFF_DEG = 8.0;
	public static final double MAX_TY_DIFF_DEG = 8.0;
	/**
	 * Cap on a group's horizontal span. Without this, transitive chaining (A near B,
	 * B near C, ...) can merge separate clusters into one group whose mean angle aims
	 * at empty floor between them.
	 */
	public static final double MAX_GROUP_SPAN_DEG = 22.0;

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
	/** Group size at which the size bonus saturates; a DECODE pattern is three artifacts. */
	public static final int SCORE_SIZE_SATURATION = 3;
	/** Total area at which the area bonus saturates (0–1 scale; ~three balls at 0.02). */
	public static final double SCORE_AREA_SATURATION = 0.06;
	/** Confidence margin above {@link #MIN_CONFIDENCE} below which a group is penalized. */
	public static final double LOW_CONFIDENCE_MARGIN = 0.15;

	// --- Target persistence ---
	/** Keep last valid target this long after vision loss (ms). */
	public static final long VISION_LOSS_TIMEOUT_MS = 350;
	/** Associate a new group with the previous target when centers are within this (deg). */
	public static final double PERSIST_ASSOCIATION_DEG = 12.0;
	/**
	 * A newly seen group must have at least this multiple of the tracked group's
	 * max member {@code ta} before we abandon the current closest lock. Stops two
	 * similarly sized piles from flipping the aimpoint every frame.
	 */
	public static final double CLOSEST_AREA_SWITCH_RATIO = 1.25;
	public static final int BALL_TARGET_SWITCH_CONFIRM_FRAMES = 3;
	public static final double BALL_TARGET_AIM_FILTER_GAIN = 0.50;
}
