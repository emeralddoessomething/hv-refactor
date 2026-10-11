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

    @Test
    void everyFunctionAndDriver() {
        assertEquals(0, AllTests.runAll(), "AllTests failed assertions");
    }
}
