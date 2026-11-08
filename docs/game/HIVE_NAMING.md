# HIVE naming and testing convention

How we name HIVE CELLs and states, and how we test detection on the real HIVE.

## 1. Field-fixed names (code, logs, strategy talk)

Use the manual's tile grid (Figure 9-5). Seen from the audience, columns run
**A–F** left to right and rows run **1–6** from the audience to the far wall.
The red HIVE sits over C3/C4 and the blue HIVE over D3/D4.

| Name | Where | Short code | Tag IDs | Its tags face |
| --- | --- | --- | --- | --- |
| Red **FAR** CELL | C4, far side | `R-F` | 30–33 ("Red Scoring Tags") | rows 5–6 |
| Red **AUDIENCE** CELL | C3, audience side | `R-A` | 34–37 | rows 1–2 |
| Blue **AUDIENCE** CELL | D3, audience side | `B-A` | 38–41 | rows 1–2 |
| Blue **FAR** CELL | D4, far side | `B-F` | 42–45 ("Blue Scoring Tags") | rows 5–6 |

A HIVE is in exactly one of two states, named after the CELL that faces up:

| State | Meaning | Code |
| --- | --- | --- |
| **FAR UP** | far CELL raised, audience CELL lowered | `HiveState.FAR_CELL_UP` |
| **AUDIENCE UP** | audience CELL raised, far CELL lowered | `HiveState.AUDIENCE_CELL_UP` |

Write both alliances' states as `R:FAR UP / B:AUD UP`.

**Why field-fixed?** "Front/back", "near/far from me", and "left/right" all
change depending on where the robot or driver is, and they flip between
alliances. FAR and AUDIENCE mean the same thing to everyone, all match.

## 2. Driver-relative hint (Driver Station display only)

Drivers stand at the side walls: red at the left wall as seen from the
audience (§9.5), blue at the right wall. The telemetry adds a hint:

| Alliance | FAR CELL is on your… | AUDIENCE CELL is on your… |
| --- | --- | --- |
| RED | LEFT | RIGHT |
| BLUE | RIGHT | LEFT |

So `Hive State Test` shows something like `YOUR HIVE (RED): FAR UP (your LEFT)`.
Code: `HiveLabels.describe(alliance, state)`.

## 3. How detection decides

A CELL's tag cluster is **RAISED** or **LOWERED** depending on its measured
height above the tiles. The cluster you can see decides the state:

| You see… | …which is | HIVE state |
| --- | --- | --- |
| FAR cluster | RAISED | FAR UP |
| FAR cluster | LOWERED | AUDIENCE UP |
| AUDIENCE cluster | RAISED | AUDIENCE UP |
| AUDIENCE cluster | LOWERED | FAR UP |

Seeing one CELL is enough, because the HIVE is a rigid see-saw. If both
clusters are visible and they disagree, that frame is thrown out.

## 4. Real-HIVE test protocol

**Rig.** Mount the Limelight on a box or stand at robot camera height, level
side to side, with a fixed upward tilt. Measure the lens height and tilt once
and enter them in `Constants.LIMELIGHT_HEIGHT_IN` / `LIMELIGHT_PITCH_DEG`.
Move the whole rig between spots without changing the camera.

**Calibrate first** with `Hive Height Calibration`. With the HIVE set FAR UP,
stand in rows 1–2, look at your AUDIENCE cluster (lowered), and press **Y** a
few times from different spots. Then stand in rows 5–6, look at your FAR
cluster (raised), and press **A**. Tip the HIVE and repeat the other way
round. Copy the suggested threshold into `HiveDetectionConfig.defaults()`.

**Score each spot** with `Hive State Test`:
1. Set the HIVE by hand. Press **D-pad up** for FAR UP or **D-pad down** for
   AUDIENCE UP.
2. Put the rig on a test spot, aimed at the HIVE. Press **A** to start a fresh
   tally and wait about 5 s.
3. Write down `Per frame` and `Confirmed`.
4. Tip the HIVE, enter the new truth with the D-pad, and repeat.

**Suggested spots.** Name each spot by its tile, so a camera on tile B1
looking at the HIVE is "B1". These are for RED; mirror the columns for blue.

| Spot | Expected view of the red HIVE |
| --- | --- |
| B1, C1, A2, C2 | AUDIENCE cluster head-on (main shooting area) |
| B6, C6, A5, C5 | FAR cluster head-on |
| A3, A4 | Side view. Expect few or no tags. |
| E2, E5 | Across the field, at long range and an angle |

**Results log**

| Spot | Truth | Cluster seen | Median height (in) | Per frame right / wrong / no vote | Confirmed right / wrong | No-3D | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| C1 | FAR UP | R-A | | | | | |
| C1 | AUD UP | R-A | | | | | |

## 5. What to expect on the real HIVE (from the HIVE model)

Building the localization model ([HIVE_LOCALIZATION.md](HIVE_LOCALIZATION.md))
showed that a TIP turns **both** of a HIVE's clusters to face the end whose CELL
is raised. Manual Figure 9-10 agrees. So:

- **From the raised end** you should see both clusters: the near one high
  (RAISED, about 49 in) and the far one low (LOWERED, about 34 in). Both votes
  agree, so expect high `right %`.
- **From the lowered end** you should see almost nothing, because the tags are
  edge-on. Expect `no vote` near 100%. The confirmed state then keeps showing
  the last answer and turns STALE, so right after a TIP, `Confirmed` can be
  wrong at these spots.
- In the results log, check whether this matches. If it does, the next
  improvement is: "pointed at my HIVE (known from the localization pose) and no
  tags visible" means the end I'm looking at is LOWERED.
