/*
 * BallHunt - one-liner ball collecting for AUTO, on Marrow Weaver.
 *
 * The same tiny, readable API the library has always had, now backed by
 * BallChaseWeaver (Weaver bezier paths + obstacle avoidance + the Limelight 3A
 * SSD detector), so an autonomous just says what it wants:
 *
 *     int picked = new BallHunt(follower, limelight, intake)
 *             .reds().collect(2).within(20)
 *             .go(this);
 *
 *   .reds()       - only red nectar
 *   .blues()      - only blue nectar
 *   .yellows()    - only neutral yellow pollen
 *   .alliance()   - red + blue (default)
 *   .everything() - red + blue + neutral yellow
 *   .collect(n)   - n balls per hunt (default 1). n <= 0 = unlimited.
 *   .within(sec)  - hard time budget before it hands back control (default 15 s)
 *   .avoid(zone)  - tell Weaver to route around something (repeatable)
 *   .go(opMode)   - BLOCKING convenience: runs the hunt (owns the drivetrain
 *                   update loop), streams telemetry, aborts cleanly, returns
 *                   how many balls it picked.
 *
 * Loop-driven alternative (advanced / when you must keep your own loop):
 *
 *     BallHunt hunt = new BallHunt(follower, limelight, intake);
 *     hunt.start();
 *     while (opModeIsActive() && !hunt.isDone()) {
 *         follower.update();       // YOU own follower.update() in this mode
 *         hunt.update();
 *         hunt.addTelemetry(telemetry);
 *         telemetry.update();
 *     }
 *     hunt.abort();
 *
 * The tracker's allowed-class filter and target lock are in effect, so the
 * robot hunts that color specifically and never flip-flops between two balls.
 */
package org.hivevisionftc.ballchase;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.skeletonarmy.marrow.zones.Zone;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallHunt {

    private final BallChaseWeaver hunt;
    private final ChaseDriver driver;

    /** Defaults: alliance colors (red+blue), 1 ball, 15 s budget. */
    public BallHunt(Follower follower, Limelight3A limelight, DcMotor intakeOrNull) {
        this(new PedroChaseDriver(follower), new BallTracker(limelight), PickupHandler.motor(intakeOrNull));
    }

    public BallHunt(Follower follower, BallTracker tracker, DcMotor intakeOrNull) {
        this(new PedroChaseDriver(follower), tracker, PickupHandler.motor(intakeOrNull));
    }

    /** Any drivetrain through the {@link ChaseDriver} seam. */
    public BallHunt(ChaseDriver driver, BallTracker tracker, PickupHandler pickupOrNull) {
        this.hunt = new BallChaseWeaver(driver, tracker, pickupOrNull);
        this.driver = driver;
        hunt.setAllowedClasses(BallTracker.CLASSES_RED_BLUE);
        hunt.setMaxPickups(1);
        hunt.setTimeBudgetSec(15.0);
    }

    public BallHunt(ChaseDriver driver, Limelight3A limelight, PickupHandler pickupOrNull) {
        this(driver, new BallTracker(limelight), pickupOrNull);
    }

    /* ---------------- which balls ---------------- */

    public BallHunt reds() {
        hunt.setAllowedClasses(BallTracker.CLASS_RED);
        return this;
    }

    public BallHunt blues() {
        hunt.setAllowedClasses(BallTracker.CLASS_BLUE);
        return this;
    }

    public BallHunt yellows() {
        hunt.setAllowedClasses(BallTracker.CLASS_YELLOW_NEUTRAL);
        return this;
    }

    public BallHunt alliance() {
        hunt.setAllowedClasses(BallTracker.CLASSES_RED_BLUE);
        return this;
    }

    public BallHunt everything() {
        hunt.setAllowedClasses(BallTracker.CLASSES_ALL);
        return this;
    }

    /* ---------------- how much / how long ---------------- */

    /** Pick up n balls on this hunt (n <= 0 = keep going). */
    public BallHunt collect(int n) {
        hunt.setMaxPickups(n);
        return this;
    }

    /** Hard time budget in seconds before handing back control. */
    public BallHunt within(double seconds) {
        hunt.setTimeBudgetSec(seconds);
        return this;
    }

    /* ---------------- how it grabs ---------------- */

    /** Swap the pickup mechanism for something other than "one motor, full power". */
    public BallHunt withPickup(PickupHandler handler) {
        hunt.setPickup(handler);
        return this;
    }

    /* ---------------- where not to go ---------------- */

    /** Tell Weaver to route around this zone (a robot, a rig, a field obstacle). */
    public BallHunt avoid(Zone zone) {
        hunt.addObstacle(zone);
        return this;
    }

    /** Replace the obstacle list. */
    public BallHunt avoid(List<Zone> zones) {
        hunt.setObstacles(zones);
        return this;
    }

    /* ---------------- run it ---------------- */

    /**
     * Blocking: run the whole hunt, own the drivetrain update loop, stream
     * telemetry, abort() when done, and return how many balls were picked.
     */
    public int go(LinearOpMode op) {
        hunt.start();
        while (op.opModeIsActive() && !hunt.isDone()) {
            driver.update();
            hunt.update();
            hunt.addTelemetry(op.telemetry);
            op.telemetry.update();
        }
        hunt.abort();
        return got();
    }

    /** Loop-driven: call once to arm, then update() each loop, then abort(). */
    public void start() {
        hunt.start();
    }

    public void update() {
        hunt.update();
    }

    public void abort() {
        hunt.abort();
    }

    /* ---------------- status ---------------- */

    public boolean isDone() {
        return hunt.isDone();
    }

    public int got() {
        return hunt.getPickups();
    }

    public String state() {
        return hunt.getState().name();
    }

    public void addTelemetry(Telemetry t) {
        hunt.addTelemetry(t);
    }

    /** The state machine underneath, for lock overrides and advanced tuning. */
    public BallChaseWeaver chase() {
        return hunt;
    }
}
