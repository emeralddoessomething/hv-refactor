package org.hivevisionftc.ballchase.tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.hivevisionftc.ballchase.BallMath;
import org.hivevisionftc.ballchase.wrapper.WrapperLogicTest;
import org.junit.jupiter.api.Test;

/** Runs the library's desktop self-test suites under Gradle (<code>./gradlew test</code>). */
class LibrarySuitesTest {

    @Test
    void ballMathRoundTrips() {
        BallMath.main(new String[0]); // throws AssertionError on failure
    }

    @Test
    void wrapperVerbLogic() {
        assertEquals(0, WrapperLogicTest.runAll(), "WrapperLogicTest failed assertions");
    }

    /** Sections S0-S3 of the self-test (no robot, no camera). S4 needs hardware and only runs as an OpMode. */
    @Test
    void headlessSelfTest() {
        assertEquals(0, Pedro3LibSelfTest.runHeadless(), "Pedro3LibSelfTest S0-S3 failed checks");
    }
}
