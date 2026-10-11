/*
 * BallTracker - perception core shared by every ball-chase driver (no-odometry
 * and Pedro-pathing variants). It turns raw Limelight 3A neural-detector output
 * into ONE chosen ball per frame, solving the two-ball flip-flop problem with a
 * target lock:
 *
 *   * allowed classes are configurable (alliance colors, with or without the
 *     neutral yellow), picked during init;
 *   * detections are gated on confidence and staleness;
 *   * while locked on a ball we STICK with whichever detection is angularly
 *     closest to where that ball was last frame (within LOCK_GATE_DEG), even if
 *     another ball is now technically nearer - so two visible balls of similar
 *     range don't make the robot oscillate;
 *   * the lock is also COLOR-bound: while a lock is alive only detections of
 *     the locked class may re-adopt it, so grab(RED) never chases a blue ball
 *     that sits nearby. The fallback can still adopt another ball, but only of
 *     the locked color, until the lock fully drops (then any allowed class).
 *   * if the locked ball can't be matched for LOCK_LOST_MS we drop the lock and
 *     fall back to nearest (lowest ty). While the lock is still fresh and the
 *     ball is momentarily missing we return a 'predicted' sighting carrying the
 *     last-known angles, so the driver keeps facing it instead of instantly
 *     re-targeting something else.
 *
 *  Usage:
 *     BallTracker tracker = new BallTracker(limelight);
 *     tracker.setAllowedClasses(pick);   // or add classes one at a time
 *     BallTracker.Sighting s = tracker.update();   // once per loop
 *
 *  Detections come from a DetectionSource (default: the Limelight; a scripted
 *  source can drive this on a laptop for unit tests). Everything here is
 *  camera-relative (tx/ty/groundRange in inches) - field projection lives in
 *  the pathing driver. All tunables are public static so you can hook a config
 *  system (e.g. Bylazar Configurables) or just edit in place.
 *
 * Class order on the 3A model: 0 = yellow_pollen (neutral), 1 = red_nectar,
 * 2 = blue_nectar. CONFIRM this against the Limelight web UI's pipeline label
 * list before trusting it - nothing here is checked against the model at
 * runtime.
 */
package org.hivevisionftc.ballchase;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class BallTracker {

    /* ---- Limelight / geometry (MEASURE THESE) ---- */
    public static double CAM_PITCH_DEG = 25.0; // camera tilt BELOW horizontal
    public static double CAM_H = 9.2; // camera lens height above floor, in
    public static double BALL_H = 3.0; // ball center height above floor, in
    public static double CAM_X_OFFSET = 0.0; // camera FORWARD of robot center, in (mount offset)
    public static double CAM_Y_OFFSET = 0.0; // camera LEFT of robot center, in; negative = right

    /* ---- class ids - CONFIRM against the pipeline label list, not this comment ---- */
    public static final int CLASS_YELLOW_NEUTRAL = 0;
    public static final int CLASS_RED = 1;
    public static final int CLASS_BLUE = 2;
    public static final Set<Integer> CLASSES_RED_BLUE = setOf(CLASS_RED, CLASS_BLUE);
    public static final Set<Integer> CLASSES_ALL = setOf(CLASS_RED, CLASS_BLUE, CLASS_YELLOW_NEUTRAL);

    /* ---- detection gating ---- */
    public static double MIN_CONF = 0.44; // CHECK telemetry: is confidence 0-1 or 0-100 on your firmware?
    public static long MAX_STALENESS_MS =
            120; // Limelight SDK reports staleness in MILLISECONDS (docs "Is The Data Fresh?")

    /* ---- target lock (stops flip-flopping when 2+ balls are visible) ---- */
    public static double LOCK_GATE_DEG = 12.0; // max frame-to-frame angular jump to count as "same ball"
    public static long LOCK_LOST_MS = 300; // drop the lock if it isn't matched for this long

    /** One chosen ball this frame. `predicted` sightings carry last-known angles
     *  while the locked ball is momentarily missing (fresh lock only). */
    public static class Sighting {
        public final int classId;
        public final double txDeg; // + to the RIGHT
        public final double tyDeg; // + UP
        public final double confidence;
        public final double distIn; // groundRange(tyDeg); 999.0 if near the horizon
        public final boolean predicted;

        public Sighting(int classId, double txDeg, double tyDeg, double confidence, boolean predicted, double distIn) {
            this.classId = classId;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.confidence = confidence;
            this.predicted = predicted;
            this.distIn = distIn;
        }
    }

    /** Off-SDK sighting datum holding exactly what the tracker consumes, so a
     *  scripted DetectionSource can drive the full lock logic on a laptop
     *  (unit tests) with no Limelight in sight. */
    public static final class RawDet {
        public final int classId;
        public final double txDeg, tyDeg, confidence, area;

        public RawDet(int classId, double txDeg, double tyDeg, double confidence, double area) {
            this.classId = classId;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.confidence = confidence;
            this.area = area;
        }
    }

    /** Feeds the tracker raw detections + staleness. The default implementation
     *  reads the Limelight; tests (and field-playback tools) script frames.
     *  latest() returns an EMPTY list (never null) when nothing is visible;
     *  stalenessMs() returns Long.MAX_VALUE when there is no data at all. */
    public interface DetectionSource {
        List<RawDet> latest();

        long stalenessMs();
    }

    private static final class LimelightSource implements DetectionSource {
        private final Limelight3A limelight;

        LimelightSource(Limelight3A limelight) {
            this.limelight = limelight;
        }

        @Override
        public List<RawDet> latest() {
            LLResult r = limelight == null ? null : limelight.getLatestResult();
            List<LLResultTypes.DetectorResult> d = r == null ? null : r.getDetectorResults();
            if (d == null || d.isEmpty()) return java.util.Collections.emptyList();
            List<RawDet> out = new java.util.ArrayList<>(d.size());
            for (LLResultTypes.DetectorResult det : d) {
                out.add(new RawDet(
                        det.getClassId(),
                        det.getTargetXDegrees(),
                        det.getTargetYDegrees(),
                        det.getConfidence(),
                        det.getTargetArea()));
            }
            return out;
        }

        @Override
        public long stalenessMs() {
            LLResult r = limelight == null ? null : limelight.getLatestResult();
            return r == null ? Long.MAX_VALUE : r.getStaleness();
        }
    }

    private final DetectionSource source;
    private Set<Integer> allowed = new HashSet<>(CLASSES_RED_BLUE);

    private Double lockTx = null, lockTy = null; // last-known angles of the locked ball
    private Integer lockClass = null; // class the lock is bound to (null = lock-free)
    private final ElapsedTime lockAge = new ElapsedTime();
    private Sighting locked = null; // last real sighting of the locked ball
    private Sighting last = null; // most recent result of update()

    public BallTracker(Limelight3A limelight) {
        this(new LimelightSource(limelight));
    }

    public BallTracker(DetectionSource source) {
        this.source = source;
    }

    /* ---------------- public API ---------------- */

    /** Which classes this tracker will chase. Replaces the whole set and clears any lock. */
    public void setAllowedClasses(Set<Integer> classes) {
        reset();
        this.allowed = new HashSet<>(classes);
    }

    public void setAllowedClasses(Integer... classes) {
        setAllowedClasses(setOf(classes));
    }

    public void addAllowedClass(int classId) {
        allowed.add(classId);
    }

    public boolean isAllowed(int classId) {
        return allowed.contains(classId);
    }

    public Set<Integer> getAllowedClasses() {
        return allowed;
    }

    /** Drop the target lock (call when starting a fresh hunt). */
    public void reset() {
        lockClass = null;
        lockTx = null;
        lockTy = null;
        locked = null;
        last = null;
    }

    /** Poll the Limelight and pick this frame's ball. Returns null only when
     *  nothing usable is visible AND no fresh lock exists to coast on. */
    public Sighting update() {
        last = compute();
        return last;
    }

    public Sighting getLast() {
        return last;
    }

    /** Staleness of the latest Limelight result in milliseconds (for scan-time gating). */
    public long getStalenessMs() {
        return source.stalenessMs();
    }

    /** Confidence- and class-gated detections for scan-time field projection.
     *  Null if the result is stale or missing. Does NOT touch the target lock. */
    public List<RawDet> getGatedDetections() {
        if (source.stalenessMs() > MAX_STALENESS_MS) return null;
        List<RawDet> dets = source.latest();
        if (dets == null || dets.isEmpty()) return null;

        List<RawDet> out = new java.util.ArrayList<>();
        for (RawDet d : dets) {
            if (!allowed.contains(d.classId)) continue;
            if (d.confidence < MIN_CONF) continue;
            out.add(d);
        }
        return out;
    }

    /** Every confident detection, whatever the class (still staleness-gated).
     *  Unlike getGatedDetections() this ignores the allowed-class filter, so an
     *  explicit "find that color" can peek at any class. Null when stale/missing. */
    public List<RawDet> getConfidentResults() {
        if (source.stalenessMs() > MAX_STALENESS_MS) return null;
        List<RawDet> dets = source.latest();
        if (dets == null || dets.isEmpty()) return null;

        List<RawDet> out = new java.util.ArrayList<>();
        for (RawDet d : dets) {
            if (d.confidence < MIN_CONF) continue;
            out.add(d);
        }
        return out;
    }

    /** Make update() chase THIS ball (class + last-known angles) exactly, for the
     *  verb wrapper's grab(target) / alignTo(target). Folds straight into the
     *  existing target lock: real detections within LOCK_GATE_DEG re-adopt it as
     *  it drifts, and a fresh lock coasts for LOCK_LOST_MS while it is missing. */
    public void lockOn(int classId, double txDeg, double tyDeg, double confidence) {
        lockClass = classId;
        lockTx = txDeg;
        lockTy = tyDeg;
        lockAge.reset();
        locked = new Sighting(classId, txDeg, tyDeg, confidence, false, groundRange(tyDeg));
    }

    /** Floor distance camera -> ball from the vertical angle. Big number if at/above the horizon. */
    public double groundRange(double tyDeg) {
        double depression = CAM_PITCH_DEG - tyDeg; // angle below horizontal to the ball
        if (depression < 1.0) return 999.0;
        return (CAM_H - BALL_H) / Math.tan(Math.toRadians(depression));
    }

    /* ---------------- camera -> field geometry (shared by every driver) ---------------- */

    /**
     * Projects a detection to a FIELD position, given the robot's field pose. This is
     * the perception half of every chase driver: the Limelight only ever reports
     * angles, so a path planner needs the pose to know where the ball actually is.
     *
     * <p>tx/ty are camera angles in degrees (tx + = ball to the RIGHT, ty + = ball UP
     * in the image). Pose angles are radians, CCW+, robot frame +X forward / +Y left.
     *
     * @param txDeg    camera horizontal angle, degrees
     * @param tyDeg    camera vertical angle, degrees
     * @param poseX    robot field X, inches
     * @param poseY    robot field Y, inches
     * @param headingRad robot field heading, radians CCW+
     * @return {fieldX, fieldY}, or null when the detection is at/above the horizon
     */
    public static double[] projectField(double txDeg, double tyDeg, double poseX, double poseY, double headingRad) {
        double pitch = Math.toRadians(CAM_PITCH_DEG);
        double h = CAM_H - BALL_H;
        double tanTx = Math.tan(Math.toRadians(txDeg));
        double tanTy = Math.tan(Math.toRadians(tyDeg));

        double denom = Math.sin(pitch) - tanTy * Math.cos(pitch);
        if (denom <= 0.02) return null; // behind/at the horizon
        double s = h / denom;

        double xr = s * (Math.cos(pitch) + tanTy * Math.sin(pitch)) + CAM_X_OFFSET; // robot frame, forward
        double yr = -s * tanTx + CAM_Y_OFFSET; // robot frame, left

        double fx = poseX + xr * Math.cos(headingRad) - yr * Math.sin(headingRad);
        double fy = poseY + xr * Math.sin(headingRad) + yr * Math.cos(headingRad);
        return new double[] {fx, fy};
    }

    /**
     * Is a ball at this field position inside the usable part of the camera image?
     * Used by scan to drop memory entries that should have been visible but were not.
     *
     * @param fieldX   ball field X, inches
     * @param fieldY   ball field Y, inches
     * @param poseX    robot field X, inches
     * @param poseY    robot field Y, inches
     * @param headingRad robot field heading, radians CCW+
     * @param maxRange ignore anything farther than this from the camera, inches
     * @return {@code true} when the ball would land inside the (margin-shrunk) frame
     */
    public static boolean inView(
            double fieldX, double fieldY, double poseX, double poseY, double headingRad, double maxRange) {
        double dxw = fieldX - poseX;
        double dyw = fieldY - poseY;
        double xr = dxw * Math.cos(headingRad) + dyw * Math.sin(headingRad);
        double yr = -dxw * Math.sin(headingRad) + dyw * Math.cos(headingRad);

        double dx = xr - CAM_X_OFFSET;
        double dy = yr - CAM_Y_OFFSET;
        if (Math.hypot(dx, dy) > maxRange) return false;

        double pitch = Math.toRadians(CAM_PITCH_DEG);
        double h = CAM_H - BALL_H;
        double zc = dx * Math.cos(pitch) + h * Math.sin(pitch);
        if (zc <= 0) return false;
        double xc = -dy;
        double yc = dx * Math.sin(pitch) - h * Math.cos(pitch);

        double tx = Math.toDegrees(Math.atan2(xc, zc));
        double ty = Math.toDegrees(Math.atan2(yc, zc));
        return Math.abs(tx) < HFOV_DEG / 2 - FOV_MARGIN_DEG && Math.abs(ty) < VFOV_DEG / 2 - FOV_MARGIN_DEG;
    }

    /* ---- field of view (used by inView) ---- */
    public static double HFOV_DEG = 54.5; // verify for your unit
    public static double VFOV_DEG = 42.0;
    public static double FOV_MARGIN_DEG = 4.0; // ignore the frame edges

    /* ---------------- selection / lock internals ---------------- */

    private Sighting compute() {
        List<RawDet> dets = source.stalenessMs() > MAX_STALENESS_MS ? null : source.latest();

        if (dets == null || dets.isEmpty()) {
            return coastOrDrop();
        }

        RawDet nearest = null; // fallback: lowest ty = closest on floor
        RawDet lockMatch = null; // best angular match to the locked ball
        double bestJump = Double.MAX_VALUE;

        for (RawDet d : dets) {
            if (!allowed.contains(d.classId)) continue;
            if (lockClass != null && d.classId != lockClass) continue; // lock is color-bound
            if (d.confidence < MIN_CONF) continue;

            if (nearest == null || d.tyDeg < nearest.tyDeg) {
                nearest = d;
            }
            if (lockTx != null) {
                double jump = Math.hypot(d.txDeg - lockTx, d.tyDeg - lockTy);
                if (jump < bestJump) {
                    bestJump = jump;
                    lockMatch = d;
                }
            }
        }

        if (nearest == null) return coastOrDrop(); // nothing confident this frame

        if (lockTx != null && lockMatch != null && bestJump <= LOCK_GATE_DEG) {
            return adopt(lockMatch); // same ball as last frame: keep it
        }
        if (locked != null && lockAge.milliseconds() <= LOCK_LOST_MS) {
            return lockedPredicted(); // fresh lock, ball missing: hold, don't flip
        }
        return adopt(nearest); // no lock, or stale: take the nearest
    }

    /** Lock onto a real detection and remember its angles + age. */
    private Sighting adopt(RawDet d) {
        lockClass = d.classId;
        lockTx = d.txDeg;
        lockTy = d.tyDeg;
        lockAge.reset();
        locked = new Sighting(d.classId, lockTx, lockTy, d.confidence, false, groundRange(lockTy));
        return locked;
    }

    /** Re-issue the locked ball's last-known angles while it is briefly missing. */
    private Sighting lockedPredicted() {
        if (locked == null) return null;
        return new Sighting(locked.classId, locked.txDeg, locked.tyDeg, locked.confidence, true, locked.distIn);
    }

    private Sighting coastOrDrop() {
        if (locked != null && lockAge.milliseconds() <= LOCK_LOST_MS) {
            return lockedPredicted();
        }
        dropLock();
        return null;
    }

    private void dropLock() {
        lockClass = null;
        lockTx = null;
        lockTy = null;
        locked = null;
    }

    private static Set<Integer> setOf(Integer... ids) {
        return new HashSet<>(Arrays.asList(ids));
    }
}
