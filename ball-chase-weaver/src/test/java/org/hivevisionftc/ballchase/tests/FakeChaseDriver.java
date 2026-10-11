package org.hivevisionftc.ballchase.tests;

import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.zones.Point;
import org.hivevisionftc.ballchase.ChaseDriver;

/** Drives the chase state machine without a camera. */
public class FakeChaseDriver implements ChaseDriver {

    public double x, y, headingRad;

    /** The plan most recently handed to follow(), or null. */
    public PathResult lastPlan;

    public double lastEndHeadingRad;
    public int followCalls;
    public int stopCalls;
    public int updateCalls;

    /** Last robot-centric powers written by drive(). */
    public double fwd, strafe, turn;

    public int driveCalls;

    /** A plan is "running" from follow() until the next update(), like a real path. */
    private boolean busy = false;

    public FakeChaseDriver(double x, double y, double headingRad) {
        this.x = x;
        this.y = y;
        this.headingRad = headingRad;
    }

    @Override
    public double x() {
        return x;
    }

    @Override
    public double y() {
        return y;
    }

    @Override
    public double headingRad() {
        return headingRad;
    }

    @Override
    public void update() {
        updateCalls++;
        busy = false; // the plan "finishes" in one tick
    }

    @Override
    public void follow(PathResult plan, double endHeadingRad) {
        this.lastPlan = plan;
        this.lastEndHeadingRad = endHeadingRad;
        this.followCalls++;
        this.busy = true;
    }

    @Override
    public boolean isBusy() {
        return busy;
    }

    @Override
    public void stop() {
        if (busy) stopCalls++;
        busy = false;
    }

    @Override
    public void drive(double fwd, double strafeLeft, double turnCcw) {
        this.fwd = fwd;
        this.strafe = strafeLeft;
        this.turn = turnCcw;
        this.driveCalls++;
    }

    /** @return the XY of the plan's endpoint, or null when nothing was planned */
    public double[] planEnd() {
        if (lastPlan == null || lastPlan.getSegments().isEmpty()) return null;
        Point p = lastPlan.getSegments().get(0).get(1.0);
        return new double[] {p.getX(), p.getY()};
    }
}
