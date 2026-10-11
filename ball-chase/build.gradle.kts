android {
    namespace = "io.github.hivevisionftc.ballchase.pedro3"
    compileSdk = 35

    defaultConfig {
        minSdk = 24
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    testOptions {
        // The suites run on the JVM; Android framework calls (Log, etc.) return defaults instead of throwing.
        unitTests.isReturnDefaultValues = true
    }

    publishing {
        singleVariant("release") {
            withSourcesJar()
            withJavadocJar()
        }
    }
}

tasks.withType<JavaCompile>().configureEach {
    options.encoding = "UTF-8"
}

tasks.withType<Javadoc>().configureEach {
    options.encoding = "UTF-8"
    // The class banners use plain-text tables and arrows; don't let strict doclint fail the release.
    (options as StandardJavadocDocletOptions).addStringOption("Xdoclint:none", "-quiet")
}

tasks.withType<Test>().configureEach {
    useJUnitPlatform()
    // The suites mutate static tunables (HiveConfig and friends), so give each test class a fresh JVM.
    forkEvery = 1
}

dependencies {
    // Supplied by the robot project at runtime: the library only compiles against them.
    // FTC SDK 12.x + Pedro Pathing 3.x
    compileOnly("org.firstinspires.ftc:RobotCore:12.0.0")
    compileOnly("org.firstinspires.ftc:Hardware:12.0.0")
    compileOnly("com.pedropathing:core:3.0.1")

    // The desktop suites need the same classes on the test classpath. Non-transitive on purpose:
    // they only touch the interfaces/stubs, not the SDK's full dependency graph.
    testImplementation("org.firstinspires.ftc:RobotCore:12.0.0") { isTransitive = false }
    testImplementation("org.firstinspires.ftc:Hardware:12.0.0") { isTransitive = false }
    testImplementation("com.pedropathing:core:3.0.1") { isTransitive = false }
    // The self-test's hardware section (S4) builds a follower through Pedro's REV Hub drivetrain/localizer.
    testImplementation("com.pedropathing:revhub:3.0.1") { isTransitive = false }
    testImplementation("org.junit.jupiter:junit-jupiter:5.10.2")
}
