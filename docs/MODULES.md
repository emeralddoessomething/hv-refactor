# Module reference — Limelight 3A ball chase

This file is the single home for the class-level documentation that used to
live in the Java file banners. Each section documents one class and is linked
from a one-line header in the source. See [README.md](../README.md) for usage
examples and the calibration table.

Jump to:

- [`BallTracker`](#balltracker--perception-core) — perception core
- [`BallChaseController`](#ballchasecontroller--no-odometry-chase) — teleop/auto state machine
- [`BallChaseWeaver`](#ballchaseweaver--weaver-auto-collect) — **current**: Weaver bezier auto collect
- [`ChaseDriver` / `PedroChaseDriver`](#chasedriver--the-drivetrain-seam) — the drivetrain seam
- [`PickupHandler`](#pickuphandler--how-a-ball-gets-grabbed) — how a ball gets grabbed
- [`BallChaseFollower`](#ballchasefollower--pedro-auto-hybrid) — archived Pedro auto hybrid
- [`BallHunt`](#ballhunt--one-line-auto-wrapper) — one-line fluent wrapper
- [`wrapper/BallWrangler`](#wrapperballwrangler--fluent-verb-api) — fluent verb API

---

## `BallTracker` — perception core

Shared by every ball-chase driver (no-odometry and Pedro-pathing variants). It
turns raw Limelight 3A neural-detector output into **one chosen ball per
frame**, solving the two-ball flip-flop problem with a target lock:

- **Allowed classes** are configurable (alliance colors, with or without the
  neutral yellow), picked during init.
- **Gating** — detections below `MIN_CONF` or older than `MAX_STALENESS_MS`
  are ignored; the SDK reports staleness in **milliseconds**.
- **Angular lock** — while locked on a ball, the tracker STICKS with whichever
  detection is angularly closest to where that ball was last frame (within
  `LOCK_GATE_DEG`), even if another ball is now technically nearer, so two
  visible balls of similar range don't make the robot oscillate.
- **Color-bound lock** — while a lock is alive only detections of the *locked
  class* may re-adopt it, so `grab(RED)` never chases a blue ball sitting
  nearby. The fallback can still adopt another ball, but only of the locked
  color, until the lock fully drops (then any allowed class).
- **Coasting** — if the locked ball can't be matched for `LOCK_LOST_MS` the
  lock drops and we fall back to nearest (lowest `ty`). While the lock is
  still fresh and the ball is momentarily missing, `update()` returns a
  `predicted` sighting with the last-known angles, so the driver keeps facing
  it instead of instantly re-targeting something else.
- **Multi-frame confirmation** (optional) — `requireConfirmation(n)` makes the
  tracker require the same ball — same class, within `CONFIRM_GATE_DEG` — be
  seen `n` CONSECUTIVE frames before adopting it, so a single-frame false
  positive (a red panel, a flicker) can't trigger a chase. While counting,
  `update()` returns `null` and `hasConfirmedTarget()` is false; once `n`
  frames are up it adopts and the normal lock takes over. `getConfirmStreak()`
  shows progress for telemetry.

**Detection source** — detections come from a `DetectionSource` (defaults to
`LimelightSource` wrapping the Limelight; a scripted source drives this on a
laptop for unit tests). `getConfidentResults()` is class-agnostic;
`getGatedDetections()` additionally applies the allowed-class filter.

**Conventions** — everything here is camera-relative (`tx`/`ty`,
`groundRange()` in inches). Field projection lives in the pathing driver
(`BallTracker.projectField` on `ball-chase-weaver`, the driver's private
`project(...)` on `ball-chase`, `wrapper/BallWrangler.projectBall`). All tunables
(`MIN_CONF`, `MAX_STALENESS_MS`, confirmation gates, `LOCK_GATE_DEG`,
`LOCK_LOST_MS`) live in `HiveConfig` (`ball-chase-legacy`) — on `ball-chase` and
`ball-chase-weaver` they are static fields on `BallTracker` itself — and are read at
point of use, so a config system (e.g. Bylazar Configurables) or plain edits can drive
them.

**Usage**

```java
BallTracker tracker = new BallTracker(limelight);
tracker.setAllowedClasses(pick);                  // or add classes one at a time
BallTracker.Sighting s = tracker.update();        // once per loop
```

**Class order on the 3A model** — `0 = yellow_pollen` (neutral),
`1 = red_nectar`, `2 = blue_nectar`. CONFIRM against the Limelight web UI's
pipeline label list before trusting it — nothing here is checked against the
model at runtime.

---

## `BallChaseController` — no-odometry chase

No-odometry (teleop or simple auto) ball chase, packaged as a reusable tool
instead of an OpMode. A state machine you drop into ANY OpMode and call every
loop; the target-lock/alliance filter is delegated to `BallTracker`, so it
behaves consistently with the pathing variant.

```
SEARCHING -> CHASING -> COASTING -> PICKUP -> SEARCHING ... -> DONE
```

| State | Behavior |
|-------|----------|
| `SEARCHING` | rotate in place (`SEARCH_TURN`) until the tracker yields a ball, or until `SEARCH_SWEEP_DEG` of dead-reckoned rotation with nothing in view — the auto guard that ends an empty-field hunt cleanly. |
| `CHASING` | aim with a P controller on `tx`, drive forward (P on dist) only once roughly aimed. |
| `COASTING` | the locked ball vanished at close range (it went under/behind the intake): keep driving straight for `COAST_MS` instead of spinning away, as long as it was lost close (`lastSeenDist <= COAST_MAX_DIST`). |
| `PICKUP` | stop, run the intake for `PICKUP_DWELL_MS`, then count the pickup — unless a `PickupConfirmer` is set and never reports a ball in the intake within `PICKUP_CONFIRM_MS` (then it's a `failedPickup`, no credit, lock dropped). |
| `DONE` | when pickups reach `maxPickups` (or `update()` is never started). |

**Usage (loop-driven)**

```java
BallChaseController chase = new BallChaseController(tracker, lf, rf, lb, rb, intake);
chase.setMaxPickups(2);              // optional: end after 2 pickups
chase.start();
while (opModeIsActive() && !chase.isDone()) {
    chase.update();
    chase.addTelemetry(telemetry);
}
chase.abort();                       // hands motors back at zero
```

**Pickup confirmation (optional)**

A blind dwell credits a pickup even when the ball bounced off the intake.
Wire in any cheap sensor — a color/infrared sensor in the intake throat, a
beam break, or intake-current spike — so a pickup only counts when a ball
actually arrived:

```java
chase.setPickupConfirmer(() -> intakeCurrentSensor.getCurrent() > 2.0);
```

A dwell that panics out without confirmation increments `getFailedPickups()`
instead of `getPickups()`, drops the tracker lock, and returns to `SEARCHING`.
Without a confirmer, every dwell is credited (legacy behavior). Timeout is
`PICKUP_CONFIRM_MS`; `0` means the robot waits forever for the sensor.

**Conventions**

- fwd `+` = forward, strafe `+` = RIGHT, turn `+` = CLOCKWISE.
- Limelight `tx` is `+` when the target is to the RIGHT, so `turn = +tx * gain`.
- Motor directions (standard symmetric mecanum): `LF F`, `LB F`, `RF R`,
  `RB R` — apply `configureMecanumDirections()` unless your drivetrain is
  flipped.
- All tunables live in `HiveConfig` (`ball-chase-legacy`) or, on `ball-chase` /
  `ball-chase-weaver`, static fields on `BallChaseController` (drive/turn/coast gains,
  search sweep, pickup timing) and are read at point of use, so a config system or
  plain edits can drive them.

---

## `BallChaseWeaver` — Weaver auto collect

**Current.** Lives in the `ball-chase-weaver` artifact and is the successor to
`BallChaseFollower`. Same perception core, same state machine, with one change
that matters: `TRAVEL` no longer drives a straight line. The scan projects
every gated Limelight detection to a FIELD position through the Pedro pose,
then [Marrow Weaver](https://github.com/Skeleton-Army/Marrow) turns the
approach point into a cubic-bezier route that bends around the obstacles you
declared, and Pedro Pathing 3 follows it.

| State | Behavior |
|-------|----------|
| `SCAN` | stop and settle, then average a few Limelight frames taken after the stop, projecting each detection (`tx`, `ty`) to a FIELD position via the drivetrain pose. Balls already in memory that are out of view stay; balls that should be visible but aren't get dropped. |
| `TRAVEL` | Weaver plans a bezier from the current pose to `APPROACH_DIST` short of the nearest remembered ball, facing it, routed around `getObstacles()`. Timeout or a refused/degenerate plan falls back to `SCAN` / `CHASE` instead of stalling. |
| `TURN` | in-place turn to face a close ball, or sweep when nothing is known. |
| `CHASE` | camera-only final approach (aim on `tx`, range from `ty`) — no field position needed, so localization error stops mattering. Coasts straight if the ball disappears right at the intake. |
| `PICKUP` | stop, run the `PickupHandler`, clear that ball from memory, back to `SCAN`. |
| `DONE` | when pickups reach `maxPickups` or the time budget runs out. |

**Obstacles** — field-inch zones, anything Marrow can describe:

```java
hunt.addObstacle(new CircleZone(new Point(72, 72), 18.0));
hunt.setObstacles(myZones);     // replace the list
hunt.clearObstacles();
```

Footprint and clearance are Marrow's global path config, so they stay
consistent with the rest of your stack:

```java
BallChaseWeaver.configurePaths(c -> c.robotWidth(16).robotHeight(16).clearance(3));
BallChaseWeaver.resetPathConfig();
```

**Extra verbs** the followers don't have: `rememberBall(x, y)` seeds a ball the
robot already knows about (planning works without seeing it), `getLastPlan()`
exposes the generated `PathResult` for telemetry, `getPlanCount()` counts
plans, and `setPickup(PickupHandler)` swaps the intake mechanism.

**Usage** — see [ball-chase-weaver/README.md](../ball-chase-weaver/README.md)
for the full walkthrough, including the Limelight 3A pipeline setup.

```java
BallChaseWeaver hunt = new BallChaseWeaver(follower, limelight, intake);
hunt.setAllowedClasses(BallTracker.CLASSES_RED_BLUE);
hunt.addObstacle(new CircleZone(new Point(72, 72), 18));
hunt.start();
while (opModeIsActive() && !hunt.isDone()) {
    follower.update();          // YOU own this
    hunt.update();
    hunt.addTelemetry(telemetry);
}
hunt.abort();
```

**Conventions** — Pedro field inches, heading radians CCW+, robot frame
+X forward / +Y left. Limelight `tx` is `+` to the RIGHT, `ty` is `+` UP.

---

## `ChaseDriver` — the drivetrain seam

The tiny interface between the chase state machine and whatever moves the
robot. `BallChaseWeaver` plans with Weaver and finishes on the camera; this is
the only thing it needs from a drivetrain: `x()`, `y()`, `headingRad()`,
`update()`, `follow(PathResult, endHeadingRad)`, `isBusy()`, `stop()`,
`drive(fwd, strafeLeft, turnCcw)`.

The shipped adapter is `PedroChaseDriver`, which wraps a Pedro 3 `Follower`:
it converts a `PathResult` with Marrow's `PedroPathingConverter`, applies the
requested end heading with `Path.linear()`, and maps robot-centric powers into
Pedro's field-centric `manual()` so the localizer keeps running during
`CHASE`. Writing your own adapter (Road Runner, a swerve base, a simulator, a
desktop unit test) takes ten lines and is the supported way to plug in another
pathing library — nothing else in the module changes.

One thing every adapter must get right: `Path.linear()` **returns a new path**.
The result is what has to reach `follow()`, or Pedro's `PathTracker` throws
`path has no heading`.

---

## `PickupHandler` — how a ball gets grabbed

Two methods, `startPickup()` / `stopPickup()`, so the library is not locked to
"one `DcMotor` running at full power". Built-ins: `PickupHandler.none()`,
`PickupHandler.motor(intake)`, `PickupHandler.motor(intake, power)` and
`PickupHandler.of(start, stop)` for the lambda form. The default constructor
form (`new BallChaseWeaver(follower, limelight, intake)`) wires the motor for
you.

---

## `BallChaseFollower` — Pedro auto hybrid

> **ARCHIVED.** Kept building and still tested so mid-season teams are not
> broken; no new features land here. The successor is
> [`BallChaseWeaver`](#ballchaseweaver--weaver-auto-collect) in the
> `ball-chase-weaver` artifact, which keeps this state machine and replaces
> the straight-line `TRAVEL` with an obstacle-avoiding Weaver bezier. The
> straight-line variant also carries a fix for a latent crash: `Path.linear()`
> returns a new path, and the result has to be the one handed to `follow()`.

Hybrid ball collection for AUTO: Pedro plans the approach, the camera
finishes. It shares the exact same perception core (`BallTracker`: allowed
classes + target lock) as the no-odometry controller, so behavior between the
two drivers is consistent.

| State | Behavior |
|-------|----------|
| `SCAN` | stop and settle, then average a few Limelight frames taken after the stop, projecting each detection (`tx`, `ty`) to a FIELD position via the Pedro pose. Balls already in memory that are out of view stay; balls that should be visible but aren't get dropped. |
| `TRAVEL` | Pedro drives to `APPROACH_DIST` short of the nearest remembered ball, facing it. A drive that times out counts as one failed approach and re-plans, giving up after `SEARCH_MAX_STEPS` failed approaches. |
| `TURN` | in-place turn to face a close ball, or sweep when nothing is known. |
| `CHASE` | camera-only final approach (aim on `tx`, range from `ty`) — no field position needed, so localization error stops mattering. Coasts straight if the ball disappears right at the intake. |
| `PICKUP` | stop, run the intake, clear that ball from memory, back to `SCAN` — or, if a `PickupConfirmer` is set and never reports a ball in the intake within `PICKUP_CONFIRM_MS`, count a failed pickup instead (no credit, drop the lock, re-plan). |
| `DONE` | when pickups reach `maxPickups` or the time budget runs out. |

**Usage**

```java
follower.update();                                   // YOU own this - we never call it
BallChaseFollower hunt = new BallChaseFollower(follower, tracker, intake);
hunt.setMaxPickups(3); hunt.setTimeBudgetSec(20);
hunt.start();
while (opModeIsActive() && !hunt.isDone()) {
    follower.update();
    hunt.update();
    hunt.addTelemetry(telemetry);
}
hunt.abort();
```

**Pickup confirmation (optional)** — same hook as the controller: wire in a
beam break, intake color/infrared sensor, or current spike so a pickup only
counts when a ball actually arrived. A dwell that panics out unconfirmed
increments `getFailedPickups()` (no credit), drops the lock, and re-plans.
Without a confirmer, every dwell is credited (legacy behavior); timeout is
`PICKUP_CONFIRM_MS` (`0` = wait forever).

```java
hunt.setPickupConfirmer(() -> intakeCurrentSensor.getCurrent() > 2.0);
```

**Conventions** — Pedro field inches, heading radians CCW+, robot frame
+X forward / +Y left. Limelight `tx` is `+` to the RIGHT, `ty` is `+` UP.
Field projection uses `HFOV_DEG`/`VFOV_DEG` plus the mount geometry
(`CAM_X_OFFSET`/`CAM_Y_OFFSET` rotated from the camera frame). Matches the
Pedro Pathing version of its artifact: 2.x on `ball-chase-legacy`, 3.x on
`ball-chase`.

---

## `BallHunt` — one-line auto wrapper

Wraps the Pedro-pathing state machine behind a tiny, readable API so an
autonomous just says what it wants. In `ball-chase-weaver` (the current
artifact) it wraps [`BallChaseWeaver`](#ballchaseweaver--weaver-auto-collect);
in the archived artifacts it wraps `BallChaseFollower`. The API is identical
across all three, so migrating is a dependency swap:

```java
BallHunt hunt = new BallHunt(follower, limelight, intake).alliance().collect(2).within(20);
int picked = hunt.go(this);          // blocking: drives, picks 2 alliance balls
// ...then go score the balls...
```

`go()` owns the drivetrain update loop, streams telemetry and aborts cleanly.
In the Weaver artifact it also owns obstacle avoidance (`.avoid(zone)`) and the
pickup mechanism (`.withPickup(handler)`), and `hunt.chase()` hands you the
underlying `BallChaseWeaver` for lock overrides and advanced tuning.

| Verb | Meaning |
|------|---------|
| `.reds()` | only red nectar |
| `.blues()` | only blue nectar |
| `.alliance()` | red + blue (default) |
| `.everything()` | red + blue + neutral yellow |
| `.collect(n)` | `n` balls per hunt (default 1); `n <= 0` = unlimited |
| `.within(sec)` | hard time budget before it hands back control (default 15 s) |
| `.confirmWhen(PickupConfirmer)` | require a physical confirmation (beam break / current spike) before crediting a pickup |
| `.gotFailed()` | how many picks were attempted but never confirmed (missed grabs) |
| `.go(opMode)` | BLOCKING convenience: runs the hunt (owns `follower.update()`), streams telemetry, aborts cleanly, returns balls picked. |

Loop-driven alternative (advanced / when you must keep your own loop):

```java
hunt.start();
while (opModeIsActive() && !hunt.isDone()) {
    follower.update();   // YOU own follower.update() in this mode
    hunt.update();
    hunt.addTelemetry(telemetry);
    telemetry.update();
}
hunt.abort();
```

The same `BallChaseFollower` semantics hold: the tracker's allowed-class filter
and target lock are in effect, so the robot hunts that color specifically and
never flip-flops between two balls.

---

## `wrapper/BallWrangler` — fluent verb API

Fluent verb API for ball collecting, drivetrain-agnostic. Subclasses
(`PedroWrangler`, `MecanumWrangler`) supply only the physical motion;
everything else — perception, deciding, intake, chaining — lives here.

An auto reads like prose:

```java
BallHunt hunt = new BallHunt(follower, limelight, intake);

if (hunt.canSee(RED)) hunt.grab(RED).within(6).go(this);
else                  hunt.scan().then(hunt.grabNearest()).go(this);

hunt.grabTwo(RED, BLUE).thenReturnTo(scorePose).go(this);
```

**Verb groups**

| Group | Verbs |
|-------|-------|
| finding (look, don't drive) | `canSee` / `count` / `findNearest` / `find` / `findLeftmost` / `findRightmost` / `findBiggest` / `findBestScore` |
| grabbing (drive + intake) | `grabNearest` / `grab(color)` / `grab(target)` / `grabLeftmost` / `grabRightmost` / `grabAll` / `grabUpTo` / `grabTwo` / `and` |
| searching | `scan` / `scanLeft` / `scanRight` / `searchAt` / `lookFor(color).orGiveUp(sec)` |
| moving around a ball | `approach(target)` / `alignTo(target)` / `backOff(inches)` / `nudge(target)` |
| intake | `intakeOn` / `intakeOff` / `reverse(sec)` / `isFull` |
| chaining / conditions | `ifSeen(color).grab(color)` / `orElse` / `then` / `thenReturnTo` / `within` / `go` |

**Semantics**

- A verb appends a step to a **one-shot** schedule. `go(LinearOpMode)` runs it
  to completion and owns the loop (for Pedro this includes
  `follower.update()`); or `start()`/`update()`/`abort()` yourself from your
  own loop, in which case YOU own the drivetrain's loop hook.
- Chains are one-shot: `go()`/`update()` leave the steps queued, so calling
  `go()` again REPLAYS them. Build a chain, run it, then `clear()` before
  scheduling the next one. Each step's timers (timeout, backOff deadline,
  tracker last-seen) start when the step first ACTUALLY runs, not when built.
- `then(other)` moves the other wrangler's already-scheduled verbs onto this
  chain; those steps stay bound to `other`, so their pickups are counted on
  `other`. Keep `orElse`/fallbacks on the **receiving** wrangler in
  cross-chain builds.
- `withPickupConfirmer(...)` installs an optional hardware confirmation
  (beam break / intake current spike): a chase only counts a pickup — and
  reports success — when it confirms a ball in the intake. Without a
  confirmer, behavior is unchanged (optimistic count).