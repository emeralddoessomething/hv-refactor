/*
 * PedroChaseDriver - the shipped ChaseDriver: Pedro Pathing 3 behind a Follower.
 *
 * Weaver plans in its own units (field inches, cubic beziers). This class is the
 * only place that knows about both libraries: it converts a PathResult with
 * Marrow's PedroPathingConverter, hands the result to the Follower, and maps
 * robot-centric powers into Pedro's field-centric manual drive so the localizer
 * keeps running during the camera-only final approach.
 *
 *     BallChaseWeaver hunt = new BallChaseWeaver(follower, limelight, intake);
 *
 * Conventions match Pedro: field inches, heading radians CCW+, robot frame
 * +X forward / +Y left.
 */
package org.hivevisionftc.ballchase;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;

public class PedroChaseDriver implements ChaseDriver {

    private final Follower follower;
    private boolean following = false;

    public PedroChaseDriver(Follower follower) {
        this.follower = follower;
    }

    /** @return the wrapped Pedro follower, for pose resets and telemetry */
    public Follower follower() {
        return follower;
    }

    @Override
    public double x() {
        return follower.pose().x();
    }

    @Override
    public double y() {
        return follower.pose().y();
    }

    @Override
    public double headingRad() {
        return follower.pose().heading();
    }

    @Override
    public void update() {
        follower.update();
    }

    @Override
    public void follow(PathResult plan, double endHeadingRad) {
        Pose start = follower.pose();
        // Shortest-way heading change, so we never spin across the +-pi seam.
        double endHeading = start.heading() + wrap(endHeadingRad - start.heading());

        // NOTE: Path.linear() returns a NEW path - the heading interpolator is
        // immutable, so the result must be the thing handed to follow().
        Path path = PedroPathingConverter.toPath(plan).linear(start.heading(), endHeading);
        follower.follow(path);
        following = true;
    }

    @Override
    public boolean isBusy() {
        return follower.isBusy();
    }

    @Override
    public void stop() {
        if (following) {
            follower.stop();
            follower.manual(DrivePowers.zero());
            following = false;
        }
    }

    @Override
    public void drive(double fwd, double strafeLeft, double turnCcw) {
        DrivePowers robotCentric = new DrivePowers(fwd, strafeLeft, turnCcw);
        follower.manual(ManualDrive.fieldCentric(robotCentric, follower.pose().heading()));
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
}
