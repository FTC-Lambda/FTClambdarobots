# HIVE localization

Code: `vision/hive/HiveLocalizer`, `HiveFieldModel`, `CameraMount`.
OpMode: `Hive Localization Test`.

The localizer estimates the robot's field position, and optionally its heading,
from the AprilTags on **both** HIVEs. It doesn't use Limelight MegaTag or a
field map (`.fmap`), because the HIVE tags move every time a HIVE tips and a
static map can't represent that.

## Field frame

| | |
| --- | --- |
| Origin | Field corner at tile **A1** (audience side, red side) |
| +x | toward column F (left to right as seen from the audience) |
| +y | toward row 6 (the far wall) |
| +z | up from the tiles |
| Heading | degrees counter-clockwise from +x. 90° faces the far wall. |
| Units | inches. Field center is (72, 72). |

The OpMode also prints the tile name (`B1`…) using the manual's grid. If Pedro
paths use a different origin or orientation, convert at the boundary. Don't
mix the two frames.

## How it works

1. Each tag's 3D range and crosshair-free `tx/ty` give its position relative to
   the robot (`CameraMount`). The Limelight's per-tag rotation estimate is never
   used, because it's noisy on 3.25 in tags at shooting range.
2. Each cluster's median height says whether that CELL is **RAISED** or
   **LOWERED**, using the same thresholds as HIVE state detection. The
   localizer therefore knows where the moving tags are on its own, for both
   alliances' HIVEs.
3. `HiveFieldModel` gives each tag's field position for that CELL position.
4. Fit:
   - **Tags only:** x, y and heading are fitted together from ≥ 2 tags at least
     4 in apart (a 2D least-squares rigid fit). A single 4-tag cluster spans
     13 in.
   - **Heading from Pinpoint:** after aligning the gyro once, every tag gives
     an x/y fix and the fixes are averaged. This is more accurate and works
     from a single tag.
5. Tags that disagree with the fit by more than 6 in are dropped once, and the
   fit is redone.

Quality numbers in telemetry:
- **fit rms:** how well the tags agree with each other. Low means good.
- **height err:** measured minus modeled tag height. If this stays large, the
  camera height or pitch, or the HIVE model, is off.
- **jitter:** standard deviation over the last 20 frames.

## HIVE model (estimated from the manual; verify on the real HIVE)

The model is derived from Figures 9-9, 9-10 and 9-15 to 9-17 and the printable
tag sheets:

| Parameter | Value | Source |
| --- | --- | --- |
| HIVE centers (x) | 59.25 (red), 84.75 (blue) | 25.5 in center to center |
| Pivot height | 43.95 in | §9.6.1 |
| Pivot → CELL bottom-face center along the arm | 15.44 in | 18.84/2 + 12.04/2 (Fig 9-9) |
| Pivot above the arm | 3.03 in | fitted, see below |
| Tilt in each stable state | 30° | Fig 9-10 |
| Tag centers within a cluster | −6.5, −2.75, +2.75, +6.5 in | measured from the printable sheets |
| ID order along x | audience clusters ascending → +x; far clusters ascending → −x | Fig 9-17 |
| Tag offset from bottom-face center | 0 in | unknown, measure it |

The 3.03 in pivot offset is chosen so the lowered CELL's outer bottom edge sits
at **30.6 in**, the manual's "Bottom of HIVE above TILES". A unit test checks
this. The model then puts cluster centers at:

| | Height | y offset from field center |
| --- | --- | --- |
| RAISED CELL cluster | **49.1 in** | ±14.9 in |
| LOWERED CELL cluster | **33.6 in** | ±11.9 in |

The default RAISED/LOWERED threshold of 40 in sits between these two heights.

**Where the tags face.** When level, a cluster faces straight down. A TIP
rotates the whole HIVE, so *both* clusters end up facing down and toward the
end whose CELL is raised. The manual's Figure 9-10 shows the same thing: seen
from the audience, the red HIVE (audience CELL up) shows two sticker rows, a
high one and a low one, and the blue HIVE (far CELL up) shows none.

What this means:
- From the **raised end** you can see both clusters of that HIVE: the near one
  high and the far one low. Both are good landmarks.
- From the **lowered end** you see that HIVE's tags almost edge-on, so expect
  no detections.
- You can always use whichever HIVE has its raised CELL toward you. If both
  HIVEs are tipped away from you, there are no tags to localize from. Rely on
  odometry there.

## Measuring the real HIVE

These measurements turn the estimates into facts. Enter them in
`HiveFieldModel.Params.defaults()`.
1. **Height of each cluster center above the tiles** in both states. The model
   predicts 49.1 and 33.6 in. If they differ, adjust `tiltDeg` and
   `pivotAboveArmIn` to match.
2. **Horizontal distance (y) from the HIVE frame's center line to each cluster
   center** in both states. The model predicts 14.9 in raised and 11.9 in
   lowered.
3. **Tag-center offset within the bottom face** (`faceOffsetIn`), if the
   clusters aren't centered on the face.
4. Check `tagOffsetsIn` with a ruler: −6.5, −2.75, +2.75, +6.5 in from the
   cluster centerline.

A quick check without measuring: put the camera stand on a tape-marked spot and
compare the OpMode's pose to the tape. If the error stays put when you move to
the other side of the HIVE, the HIVE model is off. If it changes with distance
or height, the camera mount is off.

## Using it on the robot later

- Set `LIMELIGHT_FORWARD_IN`, `LIMELIGHT_LEFT_IN` and `LIMELIGHT_YAW_DEG` in
  `Constants.java` so the pose refers to the robot's center, not the camera.
- Only correct odometry when the fix is trustworthy: tags ≥ 2, fit rms ≤ 2 in,
  height err ≤ 3 in, robot moving slowly. Blend the correction in rather than
  jumping to it.
- Pinpoint's IMU is far better than the tag-only heading. Align it once (A in
  the OpMode) and use heading-given mode from then on.
