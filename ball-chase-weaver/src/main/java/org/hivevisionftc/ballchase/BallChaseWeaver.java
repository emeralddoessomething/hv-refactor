/*
 * BallChaseWeaver - the Marrow Weaver successor to BallChaseFollower: Limelight
 * 3A detection in, smooth obstacle-avoiding bezier out, ball in the intake.
 *
 * Same battle-tested state machine as BallChaseFollower, with one change that
 * matters: TRAVEL no longer drives a straight line. The scan projects every
 * gated Limelight detection to a FIELD position through the Pedro pose, then
 * Marrow Weaver turns the approach point into a cubic-bezier route that bends
 * around the obstacles you declared, and Pedro Pathing 3 follows it.
 *
 *   SCAN    Stop and settle, average a few Limelight frames taken after the
 *           stop, and project each detection (tx, ty) to a field position.
 *           Balls already in memory that are out of view stay; balls that
 *           should be visible but aren't get dropped.
 *   TRAVEL  Weaver plans a bezier from the current pose to APPROACH_DIST short
 *           of the nearest remembered ball, facing it, routed around obstacles.
 *           Timeout or an unreachable target falls back to SCAN / CHASE.
 *   TURN    In-place turn to face a close ball, or sweep when nothing is known.
 *   CHASE   Camera-only final approach (aim on tx, range from ty) - no field
 *           position needed, so localization error stops mattering. Coasts
 *           straight if the ball disappears right at the intake.
 *   PICKUP  Stop, run the pickup handler, clear that ball from memory, SCAN.
 *   DONE    when pickups reach maxPickups or the time budget runs out.
 *
 * AUTO usage (Pedro Pathing 3):
 *     BallChaseWeaver hunt = new BallChaseWeaver(follower, limelight, intake);
 *     hunt.setAllowedClasses(BallTracker.CLASSES_RED_BLUE);
 *     hunt.addObstacle(new CircleZone(new Point(72, 72), 18));   // optional
 *     hunt.setMaxPickups(3); hunt.setTimeBudgetSec(20);
 *     hunt.start();
 *     while (opModeIsActive() && !hunt.isDone()) {
 *         follower.update();      // YOU own follower.update()
 *         hunt.update();
 *         hunt.addTelemetry(telemetry);
 *     }
 *     hunt.abort();
 *
 * One-liner form: {@link BallHunt}. Not on Pedro? Implement
 * {@link ChaseDriver} for your pathing library - nothing else changes.
 *
 * Conventions: field inches, heading radians CCW+, robot frame +X forward /
 * +Y left. Limelight tx is + to the RIGHT, ty is + UP. Class ids 0/1/2 must
 * match your Limelight pipeline's label list (yellow_pollen / red_nectar /
 * blue_nectar) - CONFIRM them in the Limelight web UI, nothing checks the model
 * at runtime. All tunables are public static so a config system can drive them.
 */
package org.hivevisionftc.ballchase;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.Zone;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallChaseWeaver {

    public enum State {
        IDLE,
        SCAN,
        TRAVEL,
        TURN,
        CHASE,
        PICKUP,
        DONE
    }

    /* ---- planning ---- */
    public static double MAX_PLAN_RANGE = 40.0; // ignore detections farther than this, in
    public static double MERGE_RADIUS = 4.0; // samples this close are the same ball
    public static double APPROACH_DIST = 24.0; // the plan stops this far short of the ball
    public static double CLEAR_RADIUS = 12.0; // balls this close to a pickup are cleared
    public static double FIELD_MIN = 6.0; // field-coordinate clip; widen for bigger fields
    public static double FIELD_MAX = 138.0;
    public static double MIN_PLAN_LENGTH = 3.0; // shorter than this: turn in place instead
    public static long TRAVEL_TIMEOUT_MS = 4000;

    /* ---- scan / search ---- */
    public static long SETTLE_MS = 250;
    public static long SCAN_MS = 250;
    public static double SEARCH_STEP_DEG = 45.0;
    public static int SEARCH_MAX_STEPS = 8;

    /* ---- in-place turn ---- */
    public static double TURN_TO_KP = 0.8; // power per radian of error
    public static double TURN_TO_MIN = 0.12;
    public static double TURN_TO_MAX = 0.5;
    public static double TURN_TO_TOL_DEG = 4.0;
    public static long TURN_TIMEOUT_MS = 2500;

    /* ---- camera-only chase ---- */
    public static double STOP_DIST = 16.0; // camera->ball floor distance at pickup
    public static double AIM_TOL_DEG = 5.0;
    public static double DRIVE_MIN_TX = 15.0; // beyond this, turn in place only
    public static double DRIVE_KP = 0.03; // forward power per inch of distance error
    public static double MIN_FWD = 0.15;
    public static double MAX_FWD = 0.7;
    public static double TURN_KP = 0.025; // power per degree of tx
    public static double MIN_TURN = 0.08;
    public static double MAX_TURN = 0.6;
    public static double COAST_MAX_DIST = 26.0;
    public static double COAST_POWER = 0.25;
    public static long COAST_MS = 450;
    public static long LOST_MS = 500; // no ball this long in chase -> forget it
    public static long PICKUP_DWELL_MS = 600;

    private static class Ball {
        double x, y;
        int n = 1;

        Ball(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    private final ChaseDriver driver;
    private final BallTracker tracker;
    private PickupHandler pickup;

    private State state = State.IDLE;
    private State afterTurn = State.SCAN;

    private List<Ball> memory = new ArrayList<>();
    private final List<Ball> samples = new ArrayList<>();
    private final List<Zone> obstacles = new ArrayList<>();
    private Ball currentTarget = null;
    private PathResult lastPlan = null;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime lastSeen = new ElapsedTime();
    private final ElapsedTime total = new ElapsedTime();
    private double lastSeenDist = Double.MAX_VALUE;
    private boolean chaseSawBall = false;
    private boolean haveChaseBall = false;
    private double lastChaseX, lastChaseY;
    private double turnTarget = 0;

    private int pickups = 0;
    private int maxPickups = 0; // 0 = unlimited
    private int searchSteps = 0;
    private int plans = 0;
    private double budgetSec = Double.MAX_VALUE;

    /* ---------------- construction ---------------- */

    /** Pedro Pathing 3 + a plain intake motor (the common case). */
    public BallChaseWeaver(Follower follower, Limelight3A limelight, DcMotor intakeOrNull) {
        this(new PedroChaseDriver(follower), new BallTracker(limelight), PickupHandler.motor(intakeOrNull));
    }

    /** Pedro Pathing 3 + an already-built tracker + a plain intake motor. */
    public BallChaseWeaver(Follower follower, BallTracker tracker, DcMotor intakeOrNull) {
        this(new PedroChaseDriver(follower), tracker, PickupHandler.motor(intakeOrNull));
    }

    /** Any drivetrain through the {@link ChaseDriver} seam + the Limelight. */
    public BallChaseWeaver(ChaseDriver driver, Limelight3A limelight, PickupHandler pickupOrNull) {
        this(driver, new BallTracker(limelight), pickupOrNull);
    }

    /**
     * The general form: a {@link ChaseDriver} adapter, a tracker (which owns the
     * Limelight) and a {@link PickupHandler}. Pass {@code null} for the pickup
     * to run the planner with nothing attached.
     */
    public BallChaseWeaver(ChaseDriver driver, BallTracker tracker, PickupHandler pickupOrNull) {
        this.driver = driver;
        this.tracker = tracker;
        this.pickup = pickupOrNull == null ? PickupHandler.none() : pickupOrNull;
    }

    /* ---------------- configuration ---------------- */

    /** Which classes to chase; delegated straight to the tracker. */
    public void setAllowedClasses(Set<Integer> classes) {
        tracker.setAllowedClasses(classes);
    }

    public void setAllowedClasses(Integer... classes) {
        tracker.setAllowedClasses(classes);
    }

    public void setMaxPickups(int n) {
        maxPickups = n;
    }

    public void setTimeBudgetSec(double s) {
        budgetSec = s;
    }

    /**
     * Swap the pickup mechanism after construction - a servo gate, a subsystem
     * object, or just a different motor power. A null handler makes the hunt
     * plan and drive without ever touching an intake.
     *
     * @param pickupOrNull the handler to run when the robot is on a ball
     */
    public void setPickup(PickupHandler pickupOrNull) {
        this.pickup = pickupOrNull == null ? PickupHandler.none() : pickupOrNull;
    }

    /**
     * Obstacles for Weaver to route around. Anything Marrow can describe works:
     * {@code CircleZone}, {@code PolygonZone}, {@code CompositeZone}. Robot
     * footprints and clearance come from Marrow's own config - see
     * {@link #configurePaths(Consumer)}.
     *
     * @param newObstacles replaces the current list; null clears it
     */
    public void setObstacles(List<Zone> newObstacles) {
        obstacles.clear();
        if (newObstacles != null) obstacles.addAll(newObstacles);
    }

    /** Add one obstacle. */
    public void addObstacle(Zone obstacle) {
        if (obstacle != null) obstacles.add(obstacle);
    }

    /** Drop every obstacle. */
    public void clearObstacles() {
        obstacles.clear();
    }

    /** @return the live obstacle list (mutate it at your own risk) */
    public List<Zone> getObstacles() {
        return Collections.unmodifiableList(obstacles);
    }

    /**
     * Seed a ball the robot already knows about - a spot remembered from an
     * earlier scan, a fixed field position, or a call from your own logic.
     * The hunt plans to it like any other sighting.
     *
     * @param fieldX ball field X, inches
     * @param fieldY ball field Y, inches
     */
    public void rememberBall(double fieldX, double fieldY) {
        merge(memory, new Ball(fieldX, fieldY));
    }

    /**
     * Tune Marrow's global path config: robot footprint, clearance, smoothing,
     * turn-cost weighting. Runs against the live config object, so it takes
     * effect on the next plan.
     *
     * <pre>{@code
     * BallChaseWeaver.configurePaths(c -> c.robotWidth(16).robotHeight(16).clearance(3));
     * }</pre>
     *
     * @param tweaks receives the current global {@link PathConfig}
     */
    public static void configurePaths(Consumer<PathConfig> tweaks) {
        if (tweaks != null) tweaks.accept(Weaver.getConfig());
    }

    /** Restore Marrow's path config to its defaults. */
    public static void resetPathConfig() {
        Weaver.resetToDefaults();
    }

    /* ---------------- status ---------------- */

    public State getState() {
        return state;
    }

    public boolean isDone() {
        return state == State.DONE;
    }

    public boolean isActive() {
        return state != State.IDLE && state != State.DONE;
    }

    public int getPickups() {
        return pickups;
    }

    /** @return how many plans have been generated this hunt */
    public int getPlanCount() {
        return plans;
    }

    /** @return the most recent Weaver plan, or null before the first TRAVEL */
    public PathResult getLastPlan() {
        return lastPlan;
    }

    /** @return how many balls the hunt currently believes are on the field */
    public int getRemembered() {
        return memory.size();
    }

    /** @return the tracker, for lock overrides ({@code tracker.lockOn(...)}) and telemetry */
    public BallTracker getTracker() {
        return tracker;
    }

    /* ---------------- lifecycle ---------------- */

    public void start() {
        pickups = 0;
        searchSteps = 0;
        plans = 0;
        lastPlan = null;
        currentTarget = null;
        tracker.reset();
        total.reset();
        enter(State.SCAN);
    }

    public void abort() {
        driver.stop();
        driver.drive(0, 0, 0);
        pickup.stopPickup();
        state = State.IDLE;
    }

    public void update() {
        if (!isActive()) return;
        if (total.seconds() > budgetSec) {
            finish();
            return;
        }

        switch (state) {
            case SCAN:
                doScan();
                break;
            case TRAVEL:
                doTravel();
                break;
            case TURN:
                doTurn();
                break;
            case CHASE:
                doChase();
                break;
            case PICKUP:
                doPickup();
                break;
            default:
                break;
        }
    }

    public void addTelemetry(Telemetry t) {
        t.addData(
                "hunt",
                "%s  pickups=%d/%s  remembered=%d  plans=%d",
                state,
                pickups,
                maxPickups > 0 ? Integer.toString(maxPickups) : "unlimited",
                memory.size(),
                plans);
        BallTracker.Sighting s = tracker.getLast();
        if (s != null) {
            t.addData(
                    "hunt target",
                    "tx=%+.1f ty=%+.1f dist=%.1f in cls=%d conf=%.2f%s",
                    s.txDeg,
                    s.tyDeg,
                    s.distIn,
                    s.classId,
                    s.confidence,
                    s.predicted ? " (predicted)" : "");
        }
        if (lastPlan != null) {
            t.addData(
                    "hunt plan",
                    "%d segment(s), %d target(s) visited, %d skipped",
                    lastPlan.getSegments().size(),
                    lastPlan.getVisitedTargets().size(),
                    lastPlan.getUnvisitedTargets().size());
        }
        for (int i = 0; i < memory.size() && i < 4; i++) {
            Ball b = memory.get(i);
            t.addData("  ball" + i, "(%.1f, %.1f) n=%d", b.x, b.y, b.n);
        }
    }

    /* ---------------- states ---------------- */

    private void doScan() {
        driver.drive(0, 0, 0);
        pickup.stopPickup();
        long t = (long) stateTimer.milliseconds();
        if (t < SETTLE_MS) return;

        if (t < SETTLE_MS + SCAN_MS) {
            sampleDetections();
            return;
        }
        finishScan();
    }

    private void sampleDetections() {
        // Only use frames captured after the stop (staleness in ms, both sides).
        if (tracker.getStalenessMs() > stateTimer.milliseconds() - SETTLE_MS) return;

        List<BallTracker.RawDet> dets = tracker.getGatedDetections();
        if (dets == null) return;

        for (BallTracker.RawDet d : dets) {
            double[] p = BallTracker.projectField(d.txDeg, d.tyDeg, driver.x(), driver.y(), driver.headingRad());
            if (p == null) continue;
            if (Math.hypot(p[0] - driver.x(), p[1] - driver.y()) > MAX_PLAN_RANGE) continue;
            merge(samples, new Ball(p[0], p[1]));
        }
    }

    private void finishScan() {
        double px = driver.x();
        double py = driver.y();

        // Keep out-of-view balls, drop ones that should be right in front of the camera.
        List<Ball> next = new ArrayList<>();
        for (Ball b : memory) {
            if (!BallTracker.inView(b.x, b.y, px, py, driver.headingRad(), MAX_PLAN_RANGE)) next.add(b);
        }
        next.addAll(samples);
        memory = next;

        if (maxPickups > 0 && pickups >= maxPickups) {
            finish();
            return;
        }

        Ball target = nearest(memory);
        if (target == null) {
            if (searchSteps >= SEARCH_MAX_STEPS) {
                finish();
                return;
            }
            searchSteps++;
            startTurn(driver.headingRad() + Math.toRadians(SEARCH_STEP_DEG), State.SCAN);
            return;
        }

        searchSteps = 0;
        currentTarget = target;
        double dx = target.x - px;
        double dy = target.y - py;
        double dist = Math.hypot(dx, dy);
        double heading = Math.atan2(dy, dx);

        if (dist > APPROACH_DIST + 4.0) {
            startTravel(target, heading);
        } else {
            startTurn(heading, State.CHASE); // already close: just face it
        }
    }

    /** Ask Marrow Weaver for a bezier to the approach point, then hand it to the driver. */
    private void startTravel(Ball target, double heading) {
        double px = driver.x();
        double py = driver.y();
        double ph = driver.headingRad();

        double ax = Range.clip(target.x - APPROACH_DIST * Math.cos(heading), FIELD_MIN, FIELD_MAX);
        double ay = Range.clip(target.y - APPROACH_DIST * Math.sin(heading), FIELD_MIN, FIELD_MAX);

        if (Math.hypot(ax - px, ay - py) < MIN_PLAN_LENGTH) {
            startTurn(heading, State.CHASE); // zero-length path guard
            return;
        }

        PathResult plan;
        try {
            plan = Weaver.builder()
                    .start(new Point(px, py, ph))
                    .end(new Point(ax, ay))
                    .obstacles(obstacles)
                    .generate();
        } catch (RuntimeException e) {
            // Planner refused (blocked target, degenerate geometry): fall back to
            // the camera finish, which needs no field path at all.
            startTurn(heading, State.CHASE);
            return;
        }

        if (plan == null || plan.getSegments().isEmpty()) {
            startTurn(heading, State.CHASE);
            return;
        }

        lastPlan = plan;
        plans++;
        // Shortest-way heading change, so we never spin across the +-pi seam.
        driver.follow(plan, ph + wrap(heading - ph));
        enter(State.TRAVEL);
    }

    private void doTravel() {
        pickup.stopPickup();
        if (!driver.isBusy()) {
            enter(State.CHASE);
        } else if (stateTimer.milliseconds() > TRAVEL_TIMEOUT_MS) {
            driver.stop(); // stuck or blocked: re-plan from wherever we are
            enter(State.SCAN);
        }
    }

    private void startTurn(double targetHeading, State next) {
        turnTarget = targetHeading;
        afterTurn = next;
        enter(State.TURN);
    }

    private void doTurn() {
        pickup.stopPickup();
        double err = wrap(turnTarget - driver.headingRad()); // + = need CCW
        if (Math.abs(err) < Math.toRadians(TURN_TO_TOL_DEG) || stateTimer.milliseconds() > TURN_TIMEOUT_MS) {
            driver.drive(0, 0, 0);
            enter(afterTurn);
            return;
        }
        double cmd = Range.clip(err * TURN_TO_KP, -TURN_TO_MAX, TURN_TO_MAX);
        if (Math.abs(cmd) < TURN_TO_MIN) cmd = Math.copySign(TURN_TO_MIN, err);
        driver.drive(0, 0, cmd);
    }

    private void doChase() {
        BallTracker.Sighting s = tracker.update();

        if (s != null && !s.predicted) {
            double tx = s.txDeg;
            double dist = s.distIn;

            lastSeen.reset();
            lastSeenDist = dist;
            chaseSawBall = true;

            double[] p = BallTracker.projectField(tx, s.tyDeg, driver.x(), driver.y(), driver.headingRad());
            if (p != null) {
                lastChaseX = p[0];
                lastChaseY = p[1];
                haveChaseBall = true;
            }

            boolean aimed = Math.abs(tx) < AIM_TOL_DEG;
            if (dist <= STOP_DIST && aimed) {
                driver.drive(0, 0, 0);
                enter(State.PICKUP);
                return;
            }

            // Pedro turns CCW+, tx is + right.
            double turn = Range.clip(-tx * TURN_KP, -MAX_TURN, MAX_TURN);
            if (!aimed && Math.abs(turn) < MIN_TURN) turn = Math.copySign(MIN_TURN, -tx);

            double fwd = 0.0;
            if (dist > STOP_DIST && Math.abs(tx) < DRIVE_MIN_TX) {
                fwd = Range.clip((dist - STOP_DIST) * DRIVE_KP, MIN_FWD, MAX_FWD);
            }
            pickup.startPickup();
            driver.drive(fwd, 0, turn);
            return;
        }

        if (s != null) {
            // Predicted: locked ball briefly missing (fresh lock). Mirror the
            // real-loss handling for close balls (coast, else pickup); otherwise
            // keep facing the last-known angle without re-planning.
            boolean wasClose = lastSeenDist <= COAST_MAX_DIST;
            if (wasClose) {
                if (lastSeen.milliseconds() < COAST_MS) {
                    pickup.startPickup();
                    driver.drive(COAST_POWER, 0, 0);
                    return;
                }
                driver.drive(0, 0, 0);
                enter(State.PICKUP);
                return;
            }
            double turn = Range.clip(-s.txDeg * TURN_KP, -MAX_TURN, MAX_TURN);
            if (Math.abs(turn) < MIN_TURN) turn = Math.copySign(MIN_TURN, -s.txDeg);
            driver.drive(0, 0, turn);
            return;
        }

        // Fully lost (lock dropped or never established).
        if (chaseSawBall && lastSeenDist <= COAST_MAX_DIST) {
            if (lastSeen.milliseconds() < COAST_MS) {
                pickup.startPickup();
                driver.drive(COAST_POWER, 0, 0); // ball went under the camera: keep going straight
            } else {
                driver.drive(0, 0, 0);
                enter(State.PICKUP);
            }
            return;
        }

        driver.drive(0, 0, 0);
        if (lastSeen.milliseconds() > LOST_MS) {
            if (currentTarget != null) memory.remove(currentTarget);
            enter(State.SCAN);
        }
    }

    private void doPickup() {
        driver.drive(0, 0, 0);
        pickup.startPickup();
        if (stateTimer.milliseconds() < PICKUP_DWELL_MS) return;

        pickups++;
        double cx, cy;
        boolean haveCoords = true;
        if (haveChaseBall) {
            cx = lastChaseX;
            cy = lastChaseY;
        } else if (currentTarget != null) {
            cx = currentTarget.x;
            cy = currentTarget.y;
        } else {
            cx = 0;
            cy = 0;
            haveCoords = false;
        }

        if (haveCoords) {
            for (int i = memory.size() - 1; i >= 0; i--) {
                Ball b = memory.get(i);
                if (Math.hypot(b.x - cx, b.y - cy) < CLEAR_RADIUS) memory.remove(i);
            }
        }
        if (currentTarget != null) memory.remove(currentTarget);
        currentTarget = null;
        haveChaseBall = false;
        pickup.stopPickup();
        tracker.reset();

        if (maxPickups > 0 && pickups >= maxPickups) {
            finish();
            return;
        }
        enter(State.SCAN);
    }

    private void finish() {
        driver.stop();
        driver.drive(0, 0, 0);
        pickup.stopPickup();
        state = State.DONE;
    }

    /* ---------------- helpers ---------------- */

    private void enter(State s) {
        state = s;
        stateTimer.reset();
        if (s == State.SCAN) samples.clear();
        if (s == State.CHASE) {
            lastSeen.reset();
            lastSeenDist = Double.MAX_VALUE;
            chaseSawBall = false;
            haveChaseBall = false;
        }
        if (s != State.TRAVEL) driver.stop(); // leaves path following for open-loop drive
    }

    private void merge(List<Ball> list, Ball b) {
        for (Ball e : list) {
            if (Math.hypot(e.x - b.x, e.y - b.y) < MERGE_RADIUS) {
                e.x = (e.x * e.n + b.x) / (e.n + 1);
                e.y = (e.y * e.n + b.y) / (e.n + 1);
                e.n++;
                return;
            }
        }
        list.add(b);
    }

    private Ball nearest(List<Ball> list) {
        double px = driver.x();
        double py = driver.y();
        Ball best = null;
        double bestD = Double.MAX_VALUE;
        for (Ball b : list) {
            double d = Math.hypot(b.x - px, b.y - py);
            if (d < bestD) {
                bestD = d;
                best = b;
            }
        }
        return best;
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
}
