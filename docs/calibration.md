# Calibration — measure the camera, and read the coordinates

Two things trip up every team that picks this library up cold: (1) how to
actually measure the calibration constants with a ruler + tape, and (2) which
coordinate system each sign belongs to. This page gives the literal procedure
for the first and a diagram for the second.

The constants below all live in one class: `HiveConfig` (`ball-chase-legacy`; on `ball-chase`
and `ball-chase-weaver` the same fields are static members of `BallTracker` /
`BallChaseController` / `BallChaseWeaver`)
(fields `CAM_*`, `HFOV_DEG`, `VFOV_DEG`, `MIN_CONF`, `LOCK_*`, `CONFIRM_*`).

## What each constant means

| Constant | Meaning |
|----------|---------|
| `CAM_H` | camera **lens center** height above the floor, inches |
| `BALL_H` | **center** of the game element above the floor, inches |
| `CAM_PITCH_DEG` | camera optic axis tilt **below horizontal**, degrees |
| `CAM_X_OFFSET` | camera **forward** of the robot center, inches |
| `CAM_Y_OFFSET` | camera **left** of the robot center, inches (+ = left, − = right) |
| `HFOV_DEG` / `VFOV_DEG` | horizontal / vertical field of view, degrees |
| `MIN_CONF` | minimum detection confidence the tracker will accept |

## How to measure each one (literal procedure)

### Camera height — `CAM_H`

Measure **floor → center of the camera lens**, not the top or bottom of the
enclosure.

```text
       CAMERA
          ●     ← measure to here
          │
          │ CAM_H
──────────┴────────── floor
```

If you can't reach the lens with the camera mounted, measure the floor → bottom
of the mount and add the mount's own height.

### Ball height — `BALL_H`

Measure **floor → center of the ball**, not its top.

```text
       ●
      /_\          ← center of the ball (the middle of its diameter)
      │ BALL_H
─────────── floor
```

For BioBuzz: `red_nectar` / `blue_nectar` / `yellow_pollen` all sit with their
centers roughly the same height — measure one on the actual field mat, where
you'll be picking them up.

### Camera pitch — `CAM_PITCH_DEG`

Measure the angle between the camera's optic axis and horizontal:
put a level against the side of the camera and measure its downward angle.

```text
          camera
            ●
             \
              \  18°   ← CAM_PITCH_DEG (below horizontal)
               \
───────────────────────── horizontal
```

A protractor on a level is plenty. Small pitch errors don't look bad on
telemetry, but they shift the distance estimate with target position — steep
mounts amplify the error.

### Camera offsets — `CAM_X_OFFSET`, `CAM_Y_OFFSET`

Measure from the robot's reference point (center of the drivetrain) to the
camera center:

```text
                 FRONT
                   ↑
             camera ●
                   │  CAM_X_OFFSET (forward, +)
                   │
             robot │ center
                   ●
```

Left/right: `CAM_Y_OFFSET` is **left-positive** from the robot's frame. If the
camera is mounted to the robot's right, `CAM_Y_OFFSET` is negative.

## Coordinate conventions (the diagram)

There are exactly four frames. Get each sign right once and the robot never
"drives backwards on you":

### 1. FTC / robot frame

```text
            +Y (strafe LEFT)
              ↑
              │
 -X ←- -0-- ROBOT --0→ +X (forward)
              │
              ↓
            -Y (strafe RIGHT)

          turn + = CLOCKWISE (top view)
```

### 2. Limelight camera frame (`tx` / `ty`)

```text
      tx < 0           tx = 0           tx > 0
        \                |                /
         \               |               /
          \              |              /
           ●─────────────●─────────────●   camera

   + ty = UP,  − ty = DOWN
```

So: `tx > 0` = ball is to the robot's RIGHT → the chase turns clockwise to
aim (a positive corrected steering command).

### 3. Pedro Pathing frame

```text
            +Y field (away from alliance wall)
              ↑
              │
              │
 ─────────────┼─────────────→ +X field (right of your alliance)
              robot

          heading + = CCW (radians)
```

The camera-relative ball position is rotated into this field frame using the
robot's heading, so a ball "forward-right" in the robot frame stays
consistent as the robot turns.

### 4. Camera mount offsets

`CAM_X_OFFSET` / `CAM_Y_OFFSET` are the camera's position in the **robot
frame** (forward = +X, left = +Y). Field projection (in `BallChaseFollower`
and the wrapper's `projectBall`) adds the camera's offset, rotated by the
heading, to the rotated ball position.

## The golden rule

```text
tx sign  →  turn direction      (verify on the bench first)
CAM_H / BALL_H / CAM_PITCH_DEG  →  distance truth (measure, don't guess)
```

Always do the three verification checks from the
[quick start](https://github.com/hivevisionftc/Hive-Vision/blob/main/docs/quick_start.md#7-verify-the-three-things-that-break-you)
(distance at 24 in, direction of `tx`, target lock) before an autonomous
routine is allowed to move the robot.

## Calibration values — an example, not a template

```java
HiveConfig.CAM_H         = 13.25;   // measured on YOUR robot
HiveConfig.BALL_H        = 2.50;    // measured on the field mat
HiveConfig.CAM_PITCH_DEG = 18.0;    // level + protractor
HiveConfig.CAM_X_OFFSET  = 4.0;     // camera 4 in forward of center
HiveConfig.CAM_Y_OFFSET  = -1.5;    // camera 1.5 in to the robot's right
HiveConfig.HFOV_DEG      = 62.0;    // your lens spec
HiveConfig.VFOV_DEG      = 48.0;    // your lens spec
```

These numbers are examples only. The single most common calibration mistake is
copying values off another team's robot.