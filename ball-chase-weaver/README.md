# Hive Ball Chase (Weaver)

Limelight 3A ball detection, target locking and **obstacle-avoiding** chase
drivers for FTC robots. This is the current artifact of the Hive Ball Chase
library: it keeps the perception core everyone already runs (`BallTracker`) and
the one-line `BallHunt`, and replaces `BallChaseFollower` with
`BallChaseWeaver`, whose travel leg is planned by [Marrow
Weaver](https://github.com/Skeleton-Army/Marrow) and followed by Pedro Pathing 3.

Part of [Hive Vision](https://github.com/hivevisionftc/Hive-Vision) (model,
training and calibration tooling).

| | `ball-chase-weaver` | `ball-chase` | `ball-chase-legacy` |
|---|---|---|---|
| Pathing | Pedro 3 + Marrow Weaver | Pedro 3, straight lines | Pedro 2, straight lines |
| Avoids obstacles | yes | no | no |
| Status | **current** | archived | archived |

Add **exactly one** of the three. They all ship the same package
(`org.hivevisionftc.ballchase`), so they cannot coexist on a robot.

## Installation

```gradle
// TeamCode/build.gradle
dependencies {
    implementation 'org.hivevisionftc:ball-chase-weaver:1.0.0'
}
```

Published to Maven Central, which the FTC Gradle setup already includes. The
library compiles against the FTC SDK and Pedro Pathing (`compileOnly`), so your
robot project supplies those: **FTC SDK 12.x + Pedro Pathing 3.x**. Marrow
comes with this artifact automatically - you do not add it yourself.

If you use other versions, compatibility is not guaranteed.

## Works with a stock FtcRobotController checkout

Verified against
[FIRST-Tech-Challenge/FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController)
master: FTC SDK 12.0.0, AGP 8.13.2, Gradle 9.1.0, Java 8, minSdk 24 - exactly what this artifact
compiles against. **No repository edits**: every dependency (FTC SDK, Pedro, Marrow, this library)
is on Maven Central, and a stock checkout already declares `mavenCentral()` + `google()`.

1. `TeamCode/build.gradle`:

   ```gradle
   dependencies {
       implementation project(':FtcRobotController')   // already there
       implementation 'com.pedropathing:core:3.0.1'
       implementation 'com.pedropathing:revhub:3.0.1'  // REV Hub drivetrain/localizer
       implementation 'org.hivevisionftc:ball-chase-weaver:1.0.0'
   }
   ```

2. Driver Station Configure app: a `Limelight3A` device named `limelight`, the intake motor named
   `intake` (any names, but the OpModes below use these).

3. OpModes: copy [`docs/examples/`](../docs/examples/) into TeamCode - `SampleFollowerFactory`
   (Pedro 3 follower construction, **TUNE ME** placeholders), `AutoBallHuntOpMode` (the one-liner)
   and `ManualBallChaseOpMode` (the full state-machine loop). They compile as-is against this
   artifact.

### Before the library is on Maven Central

Until `1.0.0` is published, install it from your own machine:

```bash
# in this repo - publishes org.hivevisionftc:ball-chase-weaver:1.0.0 to ~/.m2/repository
./gradlew :ball-chase-weaver:deployLocal
```

then in **`build.dependencies.gradle`** of FtcRobotController (TeamCode-scoped - no need to edit
the root `build.gradle`):

```gradle
repositories {
    mavenLocal()
    mavenCentral()
    google()
}
```

`mavenLocal()` can be deleted once the artifact is on Central.

## Limelight 3A: you do not train anything

Hive Vision ships the detector. Load it once on the camera and this library
consumes it:

1. Laptop on the robot Wi-Fi, open `http://limelight.local:5801`.
2. Pipeline Type: **Neural Detector**.
3. Upload `best_limelight3a_ssd_mobilenetv2_300x300.tflite` and `labels.txt`
   from the Hive Vision repo (`neural-net/weights/`). Keep the label order:
   `yellow_pollen`, `red_nectar`, `blue_nectar`.
4. Runtime engine: **CPU** (the 3A has no accelerator; "Coral" shows nothing).
5. Input resolution **300 x 300**, detection threshold **0.35 - 0.45**.
6. In the Driver Station Configure app add a `Limelight3A` device and use that
   same name in `hardwareMap`.

Verify boxes appear in the web UI stream before writing any code - if they
don't, it's the pipeline, not this library. Full walkthrough:
[Hive Vision docs](https://github.com/hivevisionftc/Hive-Vision).

Class ids are `0 = yellow_pollen`, `1 = red_nectar`, `2 = blue_nectar`. Nothing
checks the model at runtime, so CONFIRM them against your pipeline's label list.

## Quick start

The one-liner (blocking, owns the drivetrain update loop):

```java
int picked = new BallHunt(follower, limelight, intake)
        .reds().collect(2).within(20)
        .go(this);
```

The state machine, when you need to keep your own loop:

```java
BallChaseWeaver hunt = new BallChaseWeaver(follower, limelight, intake);
hunt.setAllowedClasses(BallTracker.CLASSES_RED_BLUE);
hunt.setMaxPickups(3);
hunt.setTimeBudgetSec(20);

hunt.start();
while (opModeIsActive() && !hunt.isDone()) {
    follower.update();          // YOU own follower.update()
    hunt.update();
    hunt.addTelemetry(telemetry);
    telemetry.update();
}
hunt.abort();
```

**YOU own `follower.update()`.** The hunt never calls it for you - it only
plans and drives.

## Avoiding obstacles

Declare what to steer around, in field inches. Anything Marrow can describe
works - circles, polygons, composites:

```java
import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.Point;

// a rig / a static robot / a field obstacle at (72, 72), 18" across
hunt.addObstacle(new CircleZone(new Point(72, 72), 18.0));

// or through the one-liner
int picked = new BallHunt(follower, limelight, intake)
        .avoid(new CircleZone(new Point(72, 72), 18.0))
        .collect(2)
        .go(this);
```

Robot footprint and how close is "too close" are Marrow's config, not per-hunt
numbers, so they stay consistent with the rest of your stack:

```java
BallChaseWeaver.configurePaths(c -> c.robotWidth(16).robotHeight(16).clearance(3));
```

`BallChaseWeaver.resetPathConfig()` puts it all back to Marrow's defaults.

## Not on Pedro? Bring your own drivetrain

The chase talks to a `ChaseDriver`, not to Pedro. The shipped
`PedroChaseDriver` wraps a Pedro 3 `Follower`; anything else is a seven-method
interface:

```java
public class MyRoadRunnerDriver implements ChaseDriver {
    // x(), y(), headingRad()  <- your pose
    // update()                <- one OpMode loop tick (localization + powers)
    // follow(plan, endHeadingRad)  <- convert plan.getPath() to your path type
    // isBusy(), stop(), drive(fwd, strafeLeft, turnCcw)
}

BallChaseWeaver hunt = new BallChaseWeaver(new MyRoadRunnerDriver(), limelight,
        PickupHandler.motor(intake));
```

Units are fixed so adapters stay interchangeable: field **inches**, heading
**radians CCW+**, robot frame **+X forward / +Y left**, drive powers
**+ forward / + strafe left / + turn CCW**.

## Not "one motor at full power"? Bring your own pickup

```java
hunt.setPickup(PickupHandler.motor(intake, 0.8));                    // motor + power
hunt.setPickup(PickupHandler.of(() -> gate.open(), () -> gate.close()));   // anything else
hunt.setPickup(PickupHandler.none());                               // planner-only

// one-liner form
int picked = new BallHunt(follower, limelight, intake)
        .withPickup(PickupHandler.of(() -> gate.open(), () -> gate.close()))
        .go(this);
```

## Migrating from `ball-chase` / `BallChaseFollower`

| Before | Now |
|---|---|
| `io.github...:ball-chase` / `ball-chase` dependency | `ball-chase-weaver` dependency |
| `new BallChaseFollower(follower, tracker, intake)` | `new BallChaseWeaver(follower, tracker, intake)` |
| `new BallHunt(follower, limelight, intake)` | unchanged |
| `tracker`, `setAllowedClasses`, `setMaxPickups`, `setTimeBudgetSec`, `start/update/abort/isDone/addTelemetry` | unchanged |
| straight-line `TRAVEL` | one cost-ordered Weaver route through every remembered ball, around obstacles |
| — | `addObstacle(Zone)` / `.avoid(zone)` |
| — | `ChaseDriver`, `PickupHandler` |

`BallChaseFollower` still exists in `ball-chase`, `@Deprecated`, so nothing
breaks mid-season. It also carries a fix for a latent crash in straight-line
travel (`Path.linear()` returns a new path; the result has to be the one handed
to `follow()`, or Pedro's `PathTracker` throws "path has no heading").

## How it drives

```
SCAN    stop, settle, average a few Limelight frames taken after the stop and
        project each detection (tx, ty) to a FIELD position through the Pedro
        pose. Balls out of view stay in memory; balls that should have been
        visible and weren't get dropped.
TRAVEL  Weaver plans ONE route through the APPROACH_DIST-short approach point
        of EVERY remembered ball - visit order picked by Marrow's
        TargetOrderer (distance weighted against turning; unreachable targets
        dropped) - bent around your obstacles. A remembered ball that gets
        within BREAK_TO_CHASE_IN while in view ends the route early (collect
        en route). Timeout or a fully blocked plan falls back to SCAN / CHASE.
TURN    in-place turn to face a close ball, or sweep when nothing is known.
CHASE   camera-only final approach (aim on tx, range from ty) - no field
        position needed, so localization error stops mattering. Coasts
        straight if the ball disappears right at the intake.
PICKUP  stop, run the pickup handler, clear that ball from memory, back to SCAN.
DONE    when pickups reach maxPickups or the time budget runs out.
```

## Calibrate before running

Every tunable is a `public static` field, read at point of use, so a config
system (e.g. Bylazar Configurables) or plain edits can drive them. The
measurement procedure and coordinate sign conventions are in
[docs/calibration.md](../docs/calibration.md).

| Constant | Where | Meaning |
|----------|-------|---------|
| `CAM_PITCH_DEG`, `CAM_H`, `BALL_H` | `BallTracker` | camera tilt below horizontal, lens height, ball-center height (inches) |
| `CAM_X_OFFSET`, `CAM_Y_OFFSET` | `BallTracker` | camera mount offset from robot center (forward +X, LEFT +Y, inches) |
| `HFOV_DEG` / `VFOV_DEG` | `BallTracker` | camera field of view, used to drop out-of-view memory |
| `MIN_CONF`, `MAX_STALENESS_MS` | `BallTracker` | detection gating; the 3A reports confidence 0-1, staleness in ms |
| `APPROACH_DIST`, `MAX_PLAN_RANGE` | `BallChaseWeaver` | how short of the ball each approach point sits; how far a ball may be to plan to it |
| `MAX_ROUTE_TARGETS`, `BREAK_TO_CHASE_IN` | `BallChaseWeaver` | cap on balls joined into one route (0 = all); in-view ball distance that ends the route early |
| `FIELD_MIN` / `FIELD_MAX` | `BallChaseWeaver` | clip for planned coordinates - widen for a bigger field |
| drive/turn/coast gains, search/pickup timing | `BallChaseWeaver` | everything the drivers read at point of use |

## Building and testing

```bash
./gradlew test               # desktop self-test suites (no robot, no camera)
./gradlew assembleRelease    # AAR plus sources/javadoc jars
./gradlew spotlessCheck      # formatting check (spotlessApply to format)
```

`WeaverChaseTest` runs the full state machine on the JVM against real Marrow
plans: a fake `ChaseDriver` stands in for the drivetrain and a scripted
`DetectionSource` for the Limelight. Needs a JDK 17+ and the Android SDK
(`ANDROID_HOME` or `local.properties` with `sdk.dir`).

## License

MIT, see [LICENSE](../LICENSE).
