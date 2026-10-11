android {
    namespace = "io.github.hivevisionftc.ballchase.weaver"
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
    // The suites mutate static tunables (BallTracker and friends), so give each test class a fresh JVM.
    forkEvery = 1
}

dependencies {
    // Supplied by the robot project at runtime: the library only compiles against them.
    // FTC SDK 12.x + Pedro Pathing 3.x
    compileOnly("org.firstinspires.ftc:RobotCore:12.0.0")
    compileOnly("org.firstinspires.ftc:Hardware:12.0.0")
    compileOnly("com.pedropathing:core:3.0.1")

    // Marrow supplies the Weaver path generator. Implementation (not compileOnly) on
    // purpose: one dependency on the robot side and path generation just works.
    // Non-transitive: the weaver/zones packages never touch Marrow's JSON layer, so
    // Jackson stays off the robot and cannot clash with anything else on the hub.
    implementation("com.skeletonarmyftc.marrow:core:1.2.0") { isTransitive = false }

    // The desktop suites need the same classes on the test classpath. Non-transitive on
    // purpose: they only touch the interfaces/stubs, not the SDK's full dependency graph.
    testImplementation("org.firstinspires.ftc:RobotCore:12.0.0") { isTransitive = false }
    testImplementation("org.firstinspires.ftc:Hardware:12.0.0") { isTransitive = false }
    testImplementation("com.pedropathing:core:3.0.1") { isTransitive = false }
    testImplementation("org.junit.jupiter:junit-jupiter:5.10.2")
}
