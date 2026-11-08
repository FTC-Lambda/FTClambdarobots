# BIOBUZZ presented by RTX — FTC 2026–2027 game reference

Programmer-focused digest of the **2026–2027 FIRST Tech Challenge Competition
Manual, Version TU08 (Team Update 08, November 5, 2026)**. Season theme: FIRST
CANOPY. Kickoff was September 12, 2026.

> The Competition Manual is the authority. This digest paraphrases it and cites
> rule numbers so you can check the source. Team Updates come out every Thursday,
> so re-check the manual version (`docs/game/reference/README.md`) before relying
> on a number here. Things I inferred rather than read in the manual are marked
> **(inference)**.

How this game affects our code: [CODE_IMPACT.md](CODE_IMPACT.md).
Local copies of the official documents: [reference/](reference/README.md).

---

## 1. The game in one paragraph

Two alliances of two robots collect **POLLEN** (yellow balls) and **NECTAR**
(red or blue balls). Robots **LAUNCH** elements into the upward-facing **CELL**
of their alliance's **HIVE**. When enough land in the CELL, the bi-stable HIVE
**TIPS** (20 pts each). Each TIP unlocks one more NECTAR from the human player.
In the last 60 seconds, alliances place NECTAR and POLLEN into the four
**FLOWERS** on the field walls. The alliance whose NECTAR is on top of a FLOWER
*owns* it. Robots finish by parking in their **LOADING ZONE**.

## 2. Match timing (§10.1, §10.4, Table 9-1)

| Phase | Length | Field timer | Notes |
| --- | --- | --- | --- |
| AUTO | 30 s | 2:30 → 2:00 | No driver input (G401). Select the AUTO OpMode with the **30 s timer enabled** (G305). |
| Transition | 8 s | — | **No powered movement** (G403). If INIT of your TeleOp could move an actuator, wait until TELEOP to press INIT. |
| TELEOP | 2:00 | 2:00 → 0:00 | |
| └ FLOWER window | last 60 s | 1:00 | FLOWER scoring and NECTAR-in-FLOWER become legal (G410). All remaining NECTAR may enter the field (G426). |
| └ Final cue | last 20 s | 0:20 | Audio cue only. |
| After 0:00 | — | — | Robot must be motionless until the referee signals (G404). |

There is no named "endgame" period. The 1:00 mark is when the game changes.

## 3. Scoring elements (§9.8, §10.3.1)

| Element | Size | Color | Per match | Part # |
| --- | --- | --- | --- | --- |
| **POLLEN** | ≈ 2.8 in (7.1 cm) holed polyethylene ball | Yellow | 40 | am-5851 |
| **NECTAR** | ≈ 3.6 in (9.1 cm) holed polyethylene ball | Red ×8, Blue ×8 | 16 | am-5852 |

The manual says elements are "not perfectly spherical and may vary in size."

**Staging:**
- POLLEN: 4 in each of the 4 FLOWERS (16), 4 in each GARDEN (8), and
  **4 preloaded in each robot** (16).
- NECTAR: 3 in each upward-facing CELL (6) and 5 in each alliance area (10).

There is **no randomization, motif, or pattern** to read this season, unlike
DECODE's obelisk.

## 4. Field (§9)

- 144 × 144 in, 36 tiles of 24 in. Columns **A–C are the red side** and
  **D–F are the blue side** (G304, G402). The red alliance area is on the left
  as seen from the audience (§9.5).
- **HIVE structure**: centered. One frame carries two HIVEs (red and blue),
  25.5 in apart center to center. The pivot is 43.95 in above the tiles. Each
  HIVE is a bi-stable see-saw with 2 CELLs; each CELL opening is about
  20 × 14 in and 12 in deep. One CELL faces up at a time. The upward-facing
  CELL's opening is **53.5–65.6 in above the tiles** at a 30° tilt
  (Figure 9-10). This is the launch target.
- **FLOWER** (×4, on the perimeter wall): top opening **≈ 4 in diameter at
  ≈ 21.5 in high**. The scoring volume is between the top ring and the middle
  ring. At the bottom is a retrieval opening, ≈ 3.55 in tall × 3.57 in deep,
  where POLLEN *may be removed*.
- **GARDEN** (×2): ≈ 23 × 2 in taped zone in opposite corners. Not protected.
- **LOADING ZONE** (×2): ≈ 23 × 11 in, next to each alliance area. This is the
  PARK target.
- Official geometry: the Onshape CAD and the STEP zip (see reference/).
  Dimensions in the manual are nominal ±1 in.

## 5. AprilTags (§9.9, Figures 9-15 to 9-17)

- Family **36h11**. Tag size **3.25 in = 82.55 mm**. `Constants.APRILTAG_SIZE_MM` now holds it.
- Tags come in **clusters of 4 on one sticker**, mounted on the closed (bottom)
  end of each CELL, "facing downward towards the TILES with its bottom edge
  oriented towards the center of the FIELD."

| IDs | CELL | Figure 9-17 label |
| --- | --- | --- |
| 30, 31, 32, 33 | Red CELL, far (opposite audience) side | "Red Scoring Tags" |
| 34, 35, 36, 37 | Red CELL, audience side | "Red Audience Tags" |
| 38, 39, 40, 41 | Blue CELL, audience side | "Blue Audience Tags" |
| 42, 43, 44, 45 | Blue CELL, far side | "Blue Scoring Tags" |

From left to right as seen from the audience, Figure 9-17 shows the far-side
rows as 33 32 31 30 | 45 44 43 42 and the audience-side rows as
34 35 36 37 | 38 39 40 41.

**Vision implications (inference, check against the CAD):**
- The tags sit high on a moving structure. The CELL bottoms are roughly 30–65 in
  above the tiles depending on HIVE state, so a camera will need upward pitch
  to see them from typical shooting distances.
- The HIVE rotates every time it TIPS, so which cluster is presented, and at
  what angle, changes during the match. Do not hard-code one tag as "the goal."
  Instead, identify the alliance (30–37 red, 38–45 blue) and work out which
  CELL is currently up.
- There are no wall-mounted tags this season. Use odometry (Pinpoint/Pedro) for
  field localization. Use the CELL tags only for aiming.
- R202 / G302.E: do not put anything on the robot that mimics 36h11 tags, and
  do not shine bright lights at the field. High-intensity lights are allowed
  only briefly while targeting (R202.F).

## 6. Points (Table 10-2) and ranking points (Table 10-3)

| Achievement | AUTO | TELEOP | When assessed |
| --- | --- | --- | --- |
| LEAVE: robot no longer touching the perimeter wall | 3 | — | End of AUTO |
| PARK: robot at least partly in own LOADING ZONE | 5 | 5 | End of AUTO / end of match |
| **HIVE TIP** | **20** | **20** | Live. A TIP finished before TELEOP counts as AUTO. |
| POLLEN/NECTAR left in an upward-facing CELL | — | 2 each | End of match |
| Bottom NECTAR bonus: your color is the bottom-most NECTAR in a FLOWER | — | 5 | Live, final at end |
| POLLEN/NECTAR in a FLOWER you **own** (your NECTAR is top-most) | — | 2 each | Live, final at end |
| POLLEN/NECTAR in your color's GARDEN, regardless of who placed it | — | 1 each | End of match |

| RP | Condition (regular events) | RP |
| --- | --- | --- |
| SWARM | LEAVE + PARK points ≥ **16** | 1 |
| POLLINATOR 1 | TIPS ≥ **4** | 1 |
| POLLINATOR 2 | TIPS ≥ **7** | 1 |
| WIN / TIE | | 3 / 1 |

The SWARM math: two robots × (3 LEAVE + 5 AUTO PARK + 5 TELEOP PARK) = 26 is
the maximum, and 16 is the threshold. Practically, both robots must LEAVE and
at least one must park in both periods, or both must AUTO-park.

Thresholds for regional championships and Worlds are TBA in Team Updates.
Fouls: MINOR = 5 points and MAJOR = 20 points, credited to the opponent.

Ranking tiebreakers (Table 13-1): average RP, then average match points minus
fouls, then average TIPS, then average AUTO points.

**Strategic takeaways:**
1. TIPs drive the game. They are worth 20 points each, count for two RPs, and
   are the third tiebreaker. A fast, accurate launcher into a high CELL that
   changes position is the core mechanism.
2. AUTO should end with LEAVE + PARK and ideally ≥ 1 TIP. The 4 preloaded
   POLLEN are your AUTO ammunition.
3. The last 60 seconds is a different game: precise NECTAR placement into a
   4 in hole 21.5 in up, contested for FLOWER ownership.

## 7. Rules that constrain software and mechanisms

| Rule | What it means for us |
| --- | --- |
| **G407** | Never CONTROL more than **4** scoring elements. Regularly holding 5 is likely to be judged STRATEGIC (MAJOR FOUL + YELLOW CARD), even if the robot sorts and discards extras. Count what's inside the robot and stop intaking at 4. |
| **G408** | Never CONTROL **opponent-color NECTAR**. Intake and vision must reject it. |
| **G409** | Don't catch or deflect elements spilling from a TIPPED HIVE until they touch something else. |
| **G410** | No NECTAR into a FLOWER before 1:00 (MAJOR FOUL per NECTAR). |
| **G417** | The only legal way to TIP is LAUNCHING into the upward-facing CELL. No pushing or touching the HIVE. |
| **G418** | Only POLLEN and NECTAR go into a FLOWER's top. Only POLLEN may be removed from the bottom. |
| **G405** (TU08) | No deliberate ejection of elements from the field. Misses during real scoring attempts are excused. |
| **G402** | AUTO: stay off the opponent's side (A–C vs D–F). No controlling their elements or contacting their robot there. MAJOR FOUL. |
| **G403** | Transition: no powered motion. TeleOp `init()` must not move servos or motors. |
| **G304** | Start position: fully on your side, touching the wall, not in the LOADING ZONE or FLOWER volume, in contact with exactly 4 preloaded POLLEN, motionless after INIT. |
| **R102 / R103** | Start inside an 18 in cube. Holding the start pose with servos/motors initialized by the OpMode is allowed. |
| **R105** | After start: 18 × 24 in footprint and 29 in tall. **Limits must be mechanical, not software.** |
| **R501–R505** | 8 motors + 8 servos total, 2 per hub port. Approved motors only. Servos ≤ 8 W, ≤ 4 A stall at 6 V. |
| **R701** | Control Hub (or phone + Expansion Hub) + at most one more Expansion Hub. |
| **R702** | Limelight **3A** may be reprogrammed (Table 12-9). Limelight 3G, OpenMV, and OAK-1 are listed as prohibited. |
| **R704.D** | **Only the FTC Driver Station app may stream telemetry. "FTC Dashboard, FTControl Panels, and others are prohibited."** No continuous video stream. |
| **R708** | USB cameras: single sensor only, natively supported. No stereo. |
| **R801** | No pneumatics or air-moving devices for elements. Flywheels and rollers are fine. |
| **T405** | Calibration window before the match: you may power on, INIT, and extend, but not drive or launch. |

## 8. Changes from DECODE (2025–26) that matter to this repo

| | What this repo assumes (DECODE) | BIOBUZZ |
| --- | --- | --- |
| Elements | Green / purple balls (`BallColor`) | POLLEN 2.8 in yellow; NECTAR 3.6 in red/blue (alliance-owned) |
| Launch target | Fixed goal | HIVE CELL that **moves every TIP**, opening 53.5–65.6 in high |
| AprilTags | 165.1 mm tags, IDs 20–24 (`LimelightSeekTag`) | 82.55 mm (3.25 in), IDs 30–45 in 4-tag clusters on CELLs |
| Randomization | Obelisk motif (IDs 21–23) | None |
| Possession limit | — | 4 (G407) |
| Placement task | — | FLOWERS (4 in opening, 21.5 in high), last 60 s |
| SDK | 11.1 (`build.dependencies.gradle`) | **v12.0** is the official 2026/27 release (Sept 12, 2026). It needs Android Studio Narwhal 3 Feature Drop or newer. R701 recommends staying current. |

The manual doesn't compare BIOBUZZ with DECODE. The left column comes from this
repository's code, not from the DECODE manual.
