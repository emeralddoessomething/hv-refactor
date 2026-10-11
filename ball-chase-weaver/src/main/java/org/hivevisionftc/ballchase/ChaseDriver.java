/*
 * ChaseDriver - the tiny seam between a chase state machine and whatever moves
 * the robot. BallChaseWeaver plans with Marrow Weaver and finishes on the
 * camera; this interface is the only thing it needs from a drivetrain.
 *
 * The shipped adapter is PedroChaseDriver (Pedro Pathing 3 + any Pedro
 * drivetrain). Writing your own takes ten lines and is the supported way to
 * plug in Road Runner, a swerve base, a simulator, or a desktop unit test -
 * nothing else in the library has to change.
 *
 * Frames and units are fixed so adapters stay interchangeable:
 *   field      inches, heading radians CCW+, origin wherever the localizer says
 *   drive()    robot-centric powers: + forward, + strafe LEFT, + turn CCW,
 *              each nominally -1..1
 */
package org.hivevisionftc.ballchase;

import com.skeletonarmy.marrow.weaver.PathResult;

public interface ChaseDriver {

    /** @return the robot's field X, inches */
    double x();

    /** @return the robot's field Y, inches */
    double y();

    /** @return the robot's field heading, radians CCW+ */
    double headingRad();

    /**
     * Advance the drivetrain one loop tick: run localization, evaluate the
     * running plan, push powers to the motors. Call it every OpMode loop,
     * BEFORE the chase's {@code update()}. (Pedro users: this is
     * {@code follower.update()}.)
     */
    void update();

    /**
     * Drive a generated Weaver plan. Adapters own the conversion into whatever
     * path type their pathing library speaks; the plan already carries the
     * obstacle-avoiding bezier geometry.
     *
     * @param plan          the plan from Marrow Weaver
     * @param endHeadingRad heading to hold at the end of the plan, radians CCW+
     */
    void follow(PathResult plan, double endHeadingRad);

    /** @return {@code true} while a plan handed over by {@link #follow} is still running */
    boolean isBusy();

    /** Cancel any running plan and leave the drivetrain stopped. */
    void stop();

    /**
     * Robot-centric open-loop drive, used by the in-place turn and the
     * camera-only final approach (the states where a path would be overkill).
     *
     * @param fwd         + forward
     * @param strafeLeft  + strafe left
     * @param turnCcw     + turn counter-clockwise
     */
    void drive(double fwd, double strafeLeft, double turnCcw);
}
