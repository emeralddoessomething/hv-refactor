package org.hivevisionftc.ballchase.tests;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.hivevisionftc.ballchase.BallTracker;

/** A DetectionSource the test paints by hand, so the full lock/scan logic runs with no Limelight. */
public class ScriptedVision implements BallTracker.DetectionSource {

    public List<BallTracker.RawDet> latest = new ArrayList<>();
    public long staleness = 0;

    public ScriptedVision clear() {
        latest = new ArrayList<>();
        return this;
    }

    public ScriptedVision see(BallTracker.RawDet... dets) {
        latest = new ArrayList<>(Arrays.asList(dets));
        return this;
    }

    public ScriptedVision stale(long ms) {
        staleness = ms;
        return this;
    }

    /** A ball on the floor {@code distIn} ahead of a camera pointing along +X. */
    public static BallTracker.RawDet ballAhead(double distIn, int classId) {
        double depression = Math.toDegrees(Math.atan((BallTracker.CAM_H - BallTracker.BALL_H) / distIn));
        double ty = BallTracker.CAM_PITCH_DEG - depression;
        return new BallTracker.RawDet(classId, 0.0, ty, 0.90, 5.0);
    }

    @Override
    public List<BallTracker.RawDet> latest() {
        return latest;
    }

    @Override
    public long stalenessMs() {
        return staleness;
    }
}
