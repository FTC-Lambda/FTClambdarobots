# Pedro Pathing configuration and tuning

Pedro Pathing is installed from `com.pedropathing:ftc:2.1.2`. The registered `Tuning` TeleOp creates a mecanum `Follower` with a goBILDA Pinpoint localizer.

There is no Pedro path-based competition autonomous yet. The current `AprilTag Seek` autonomous uses the regular `Drivetrain` and Limelight instead.

## Active configuration

| Setting | Value in code |
| --- | --- |
| Drive motors | `top_left`, `back_left`, `top_right`, `back_right` |
| Motor directions | all `FORWARD` |
| Forward velocity | `65.48 in/s` |
| Lateral velocity | `56.8 in/s` |
| Forward zero-power acceleration | `-34.6` |
| Lateral zero-power acceleration | `-48.7` |
| Localizer | goBILDA Pinpoint named `pinpoint` |
| Distance unit | inches |
| Encoder pod model | goBILDA 4-bar pod |
| Forward pod Y offset | `-3.55 in` |
| Strafe pod X offset | `0.91 in` |
| Forward encoder direction | `FORWARD` |
| Strafe encoder direction | `REVERSED` |
| Path constraints | `(0.99, 100, 1, 1)` |

The values live in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java`.

## Setup and tuning order

1. Complete `docs/ai/hardware.md`, including the `pinpoint` device.
2. In the Driver Station, select `Pedro Pathing` then `Tuning`.
3. Use `Localization Test` first. Confirm forward movement increases X, right strafe increases Y, and heading rotates in the expected direction.
4. Use `Offsets Tuner`, then verify with the forward, lateral, and turn tuners.
5. Tune forward and lateral velocity, then their zero-power accelerations.
6. Use the line, triangle, and circle tests as final validation.
7. Copy each measured value into `pedroPathing/Constants.java`, rebuild, and repeat validation tests.

## Measurement status

The code still labels `forwardPodY` and `strafePodX` as values to adjust from robot measurements. Treat `-3.55` and `0.91` as current provisional settings, not verified mechanical dimensions.

The velocity and zero-power-acceleration values are marked as tuned in code, but should be revalidated after any drivetrain, wheel, battery, or weight change.

Pedro tuner examples initialize around `(72, 72)` inches. That is a tuning/test coordinate convention, not a documented match-autonomous start pose.
