/*
 * PickupHandler - how the hunt actually grabs a ball. Decoupled so the library
 * is not locked to "one DcMotor running at full power": a servo gate, a
 * roller bar with its own OpMode timing, or a subsystem object can all be a
 * three-line lambda.
 *
 *     hunt.withPickup(() -> intake.spin(), () -> intake.stop());
 *     hunt.withPickup(PickupHandler.motor(intake));      // the usual case
 *     hunt.withPickup(PickupHandler.none());             // planning demo / no intake yet
 *
 * BallChaseWeaver calls startPickup() when it stops on the ball and
 * stopPickup() before it moves again (including on abort()).
 */
package org.hivevisionftc.ballchase;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface PickupHandler {

    /** Begin collecting: run the intake, open the gate, drop the arm, ... */
    void startPickup();

    /** Stop collecting. Always called, including on abort(). */
    void stopPickup();

    /** A handler that does nothing - useful for testing the planner on the bench. */
    static PickupHandler none() {
        return new PickupHandler() {
            @Override
            public void startPickup() {}

            @Override
            public void stopPickup() {}
        };
    }

    /**
     * The common case: one motor, one power, forwards while collecting.
     *
     * @param motor the intake motor; a null motor is allowed and makes this a no-op
     * @return a handler that sets the motor's power
     */
    static PickupHandler motor(final DcMotor motor) {
        return motor(motor, 1.0);
    }

    /**
     * The common case with an explicit power.
     *
     * @param motor the intake motor; a null motor is allowed and makes this a no-op
     * @param power power to run while collecting (negative reverses the intake)
     * @return a handler that sets the motor's power
     */
    static PickupHandler motor(final DcMotor motor, final double power) {
        return new PickupHandler() {
            @Override
            public void startPickup() {
                if (motor != null) motor.setPower(power);
            }

            @Override
            public void stopPickup() {
                if (motor != null) motor.setPower(0);
            }
        };
    }

    /** Two-method convenience for the lambda form: {@code (start, stop) -> ...}. */
    static PickupHandler of(final Runnable start, final Runnable stop) {
        return new PickupHandler() {
            @Override
            public void startPickup() {
                start.run();
            }

            @Override
            public void stopPickup() {
                stop.run();
            }
        };
    }
}
