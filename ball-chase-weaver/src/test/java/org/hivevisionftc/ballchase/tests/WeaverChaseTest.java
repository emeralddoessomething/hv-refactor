package org.hivevisionftc.ballchase.tests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PathRoute;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.weaver.WeaverGenerator;
import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.Zone;
import java.util.List;
import org.hivevisionftc.ballchase.BallChaseWeaver;
import org.hivevisionftc.ballchase.BallChaseWeaver.State;
import org.hivevisionftc.ballchase.BallTracker;
import org.hivevisionftc.ballchase.PickupHandler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Desktop self-test for the Weaver chase: no robot, no camera, no Android.
 * A {@link FakeChaseDriver} stands in for the drivetrain and a
 * {@link ScriptedVision} for the Limelight, so the whole state machine runs
 * against real Marrow Weaver plans.
 */
class WeaverChaseTest {

    private static final double FAR_IN = 35.0; // distance used by the "seen while scanning" ball
    private static final double NEAR_IN = 14.0; // distance used by the "seen while chasing" ball

    private ScriptedVision vision;
    private FakeChaseDriver driver;
    private RecordingPickup pickup;

    private final double[] weaverTune = {
        BallChaseWeaver.SETTLE_MS, BallChaseWeaver.SCAN_MS, BallChaseWeaver.TRAVEL_TIMEOUT_MS,
        BallChaseWeaver.TURN_TIMEOUT_MS, BallChaseWeaver.SEARCH_STEP_DEG, BallChaseWeaver.PICKUP_DWELL_MS,
        BallChaseWeaver.COAST_MS, BallChaseWeaver.LOST_MS, BallChaseWeaver.MAX_PLAN_RANGE,
        BallChaseWeaver.APPROACH_DIST, BallChaseWeaver.MIN_PLAN_LENGTH
    };
    private final int[] weaverTuneInt = {BallChaseWeaver.SEARCH_MAX_STEPS};

    @BeforeEach
    void tune() {
        // Shrink the clock so a full hunt runs in well under a second.
        BallChaseWeaver.SETTLE_MS = 10;
        BallChaseWeaver.SCAN_MS = 10;
        BallChaseWeaver.TRAVEL_TIMEOUT_MS = 500;
        BallChaseWeaver.TURN_TIMEOUT_MS = 60;
        BallChaseWeaver.SEARCH_STEP_DEG = 90;
        BallChaseWeaver.SEARCH_MAX_STEPS = 2;
        BallChaseWeaver.PICKUP_DWELL_MS = 20;
        BallChaseWeaver.COAST_MS = 30;
        BallChaseWeaver.LOST_MS = 60;
        BallChaseWeaver.MAX_PLAN_RANGE = 40;
        BallChaseWeaver.APPROACH_DIST = 24;
        BallChaseWeaver.MIN_PLAN_LENGTH = 3;

        vision = new ScriptedVision();
        driver = new FakeChaseDriver(24, 24, 0.0);
        pickup = new RecordingPickup();
    }

    @AfterEach
    void restore() {
        BallChaseWeaver.SETTLE_MS = (long) weaverTune[0];
        BallChaseWeaver.SCAN_MS = (long) weaverTune[1];
        BallChaseWeaver.TRAVEL_TIMEOUT_MS = (long) weaverTune[2];
        BallChaseWeaver.TURN_TIMEOUT_MS = (long) weaverTune[3];
        BallChaseWeaver.SEARCH_STEP_DEG = weaverTune[4];
        BallChaseWeaver.PICKUP_DWELL_MS = (long) weaverTune[5];
        BallChaseWeaver.COAST_MS = (long) weaverTune[6];
        BallChaseWeaver.LOST_MS = (long) weaverTune[7];
        BallChaseWeaver.MAX_PLAN_RANGE = weaverTune[8];
        BallChaseWeaver.APPROACH_DIST = weaverTune[9];
        BallChaseWeaver.MIN_PLAN_LENGTH = weaverTune[10];
        BallChaseWeaver.SEARCH_MAX_STEPS = weaverTuneInt[0];
        BallChaseWeaver.resetPathConfig();
    }

    /* ---------------- geometry ---------------- */

    @Test
    void projectsABallStraightAheadToTheField() {
        double ty = ScriptedVision.ballAhead(FAR_IN, BallTracker.CLASS_RED).tyDeg;
        double[] p = BallTracker.projectField(0.0, ty, 24, 24, 0.0);
        assertNotNull(p);
        assertEquals(24 + FAR_IN, p[0], 0.5, "ball should sit FAR_IN ahead on +X");
        assertEquals(24.0, p[1], 0.5, "tx=0 means no lateral offset");

        // Same detection, robot turned 90 deg CCW: the ball moves onto +Y.
        double[] q = BallTracker.projectField(0.0, ty, 24, 24, Math.PI / 2);
        assertNotNull(q);
        assertEquals(24.0, q[0], 0.5);
        assertEquals(24 + FAR_IN, q[1], 0.5);
    }

    @Test
    void horizonDetectionsDoNotProject() {
        assertNull(BallTracker.projectField(0.0, 30.0, 0, 0, 0.0), "a ball above the horizon has no floor position");
    }

    @Test
    void inViewRejectsWhatIsBehindTheRobot() {
        double far = 100.0;
        assertTrue(BallTracker.inView(24 + FAR_IN, 24, 24, 24, 0.0, far), "dead ahead is in view");
        assertFalse(BallTracker.inView(24 - FAR_IN, 24, 24, 24, 0.0, far), "behind the camera is not");
    }

    /* ---------------- end to end ---------------- */

    @Test
    void huntsOneBallEndToEnd() {
        vision.see(ScriptedVision.ballAhead(FAR_IN, BallTracker.CLASS_RED));
        BallChaseWeaver hunt = newHunt();

        runUntil(hunt, State.TRAVEL, 3000);

        assertEquals(1, driver.followCalls, "TRAVEL should hand exactly one Weaver plan to the driver");
        assertNotNull(driver.lastPlan);
        assertEquals(1, driver.lastPlan.getSegments().size(), "one obstacle-free hop is one segment");
        double[] end = driver.planEnd();
        assertNotNull(end);
        // Ball lands at (24+FAR_IN, 24); the plan stops APPROACH_DIST short of it.
        assertEquals(24 + FAR_IN - BallChaseWeaver.APPROACH_DIST, end[0], 1.0, "plan ends at the approach point");
        assertEquals(24.0, end[1], 1.0, "plan stays on the ball's line");
        assertEquals(1, hunt.getPlanCount());

        // Now the ball is close enough for the camera-only finish.
        vision.see(ScriptedVision.ballAhead(NEAR_IN, BallTracker.CLASS_RED));
        runUntil(hunt, State.DONE, 3000);

        assertEquals(1, hunt.getPickups());
        assertEquals(0, hunt.getRemembered(), "the collected ball leaves memory");
        assertTrue(pickup.started > 0, "the intake ran during the pickup");
        assertEquals(0.0, pickup.lastPower, 1e-9, "the intake is off when the hunt ends");
    }

    @Test
    void routesAroundDeclaredObstacles() {
        // A 4" obstacle sitting right on the straight line to the approach point.
        BallChaseWeaver.configurePaths(c -> c.robotWidth(1).robotHeight(1).clearance(0.5));
        Zone obstacle = new CircleZone(new Point(30, 27), 4.0);

        vision.see(ScriptedVision.ballAhead(FAR_IN, BallTracker.CLASS_RED));
        BallChaseWeaver hunt = newHunt();
        hunt.addObstacle(obstacle);

        runUntil(hunt, State.TRAVEL, 3000);

        PathResult plan = hunt.getLastPlan();
        assertNotNull(plan, "a plan was produced despite the obstacle");
        PathRoute route = plan.getPath();
        assertTrue(
                WeaverGenerator.isPathClear(route, hunt.getObstacles(), Weaver.getConfig()),
                "the generated route keeps the configured clearance from the obstacle");

        // And it actually bent: no sample may sit on the obstacle centre.
        double closest = Double.MAX_VALUE;
        for (Point p : route.getSegments().get(0).sample(41)) {
            closest = Math.min(closest, Math.hypot(p.getX() - 30, p.getY() - 27));
        }
        assertTrue(closest > 4.0, "the route bends around the obstacle instead of clipping it, closest=" + closest);
    }

    @Test
    void aCloseBallSkipsPlanningEntirely() {
        // Ball already inside the approach distance: face it, no path, no planner.
        vision.see(ScriptedVision.ballAhead(NEAR_IN + 4, BallTracker.CLASS_BLUE));
        BallChaseWeaver hunt = newHunt();

        runUntil(hunt, State.CHASE, 3000);

        assertEquals(0, driver.followCalls, "no plan for a ball that is already close");
        assertNull(hunt.getLastPlan());
        assertEquals(0, hunt.getPlanCount());
    }

    @Test
    void searchSpinEndsAnEmptyFieldCleanly() {
        BallChaseWeaver.SEARCH_MAX_STEPS = 2;
        vision.clear();
        BallChaseWeaver hunt = newHunt();

        runUntil(hunt, State.DONE, 3000);

        assertTrue(hunt.getPickups() == 0);
        assertTrue(driver.followCalls == 0, "nothing to plan to");
        assertTrue(driver.driveCalls > 0, "the hunt swept the field looking for something");
    }

    @Test
    void timeBudgetBeatsTheStateMachine() {
        vision.see(ScriptedVision.ballAhead(FAR_IN, BallTracker.CLASS_RED));
        BallChaseWeaver hunt = newHunt();
        hunt.setTimeBudgetSec(0.001);

        runUntil(hunt, State.DONE, 3000);

        assertTrue(hunt.isDone());
        assertEquals(0, hunt.getPickups());
    }

    @Test
    void staleFramesNeverBecomePlanTargets() {
        vision.see(ScriptedVision.ballAhead(FAR_IN, BallTracker.CLASS_RED)).stale(10_000);
        BallChaseWeaver hunt = newHunt();

        runUntil(hunt, State.DONE, 3000);

        assertEquals(0, driver.followCalls, "stale detections must not become paths");
        assertEquals(0, hunt.getRemembered());
    }

    @Test
    void rememberedBallsArePlannedToWithoutSeeingThem() {
        // Far enough that scan's "should have seen it" check leaves it in memory.
        double far = BallChaseWeaver.MAX_PLAN_RANGE + 10;
        vision.clear();
        BallChaseWeaver hunt = newHunt();
        hunt.rememberBall(24 + far, 24); // e.g. spotted in a previous auto segment

        runUntil(hunt, State.TRAVEL, 3000);

        assertEquals(1, driver.followCalls, "a remembered ball is enough to plan");
        double[] end = driver.planEnd();
        assertNotNull(end);
        assertEquals(24 + far - BallChaseWeaver.APPROACH_DIST, end[0], 1.0);
    }

    /* ---------------- the Pedro seam ---------------- */

    @Test
    void convertedPathsCarryAHeadingAndResolveSegments() {
        // The single most important promise of PedroChaseDriver.follow(): the path it
        // hands the Follower has a heading interpolator, so PathTracker can resolve
        // segments instead of throwing "path has no heading".
        PathResult plan =
                Weaver.builder().start(new Point(0, 0, 0)).end(new Point(30, 0)).generate();

        Path path = PedroPathingConverter.toPath(plan).linear(0.0, Math.PI / 2);

        List<PathSegment> segments = path.getSegments();
        assertFalse(segments.isEmpty(), "getSegments() must not throw for a followed path");

        Pose end = path.endPose();
        assertEquals(30.0, end.x(), 0.5, "converted path ends on the destination");
        assertEquals(0.0, end.y(), 0.5);
        assertEquals(Math.PI / 2, path.heading(1.0), 1e-6, "the requested end heading survives conversion");
    }

    @Test
    void pickupHandlerRunsAndStopsAMotor() {
        MockMotor motor = new MockMotor();
        PickupHandler handler = PickupHandler.motor(motor, 0.8);

        handler.startPickup();
        assertEquals(0.8, motor.power, 1e-9);
        handler.stopPickup();
        assertEquals(0.0, motor.power, 1e-9);

        PickupHandler.motor(null).startPickup(); // null motor is legal, not an NPE
    }

    /* ---------------- harness ---------------- */

    private BallChaseWeaver newHunt() {
        BallChaseWeaver hunt = new BallChaseWeaver(driver, new BallTracker(vision), pickup);
        hunt.setAllowedClasses(BallTracker.CLASSES_ALL);
        hunt.setMaxPickups(1);
        hunt.start();
        return hunt;
    }

    /** Drive the machine the way an OpMode would until it settles in {@code want} (or gives up). */
    private void runUntil(BallChaseWeaver hunt, State want, long timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (System.currentTimeMillis() < deadline) {
            driver.update();
            hunt.update();
            if (hunt.getState() == want) return;
            if (hunt.getState() == State.DONE && want != State.DONE) return;
            try {
                Thread.sleep(2);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
        throw new AssertionError("timed out in " + hunt.getState() + ", wanted " + want);
    }

    /** A PickupHandler that records calls instead of moving a motor. */
    private static final class RecordingPickup implements PickupHandler {
        int started;
        int stopped;
        double lastPower = Double.NaN;

        @Override
        public void startPickup() {
            started++;
            lastPower = 1.0;
        }

        @Override
        public void stopPickup() {
            stopped++;
            lastPower = 0.0;
        }
    }
}
