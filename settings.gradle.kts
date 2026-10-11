pluginManagement {
	repositories {
		gradlePluginPortal()
		mavenCentral()
		google()
	}
}

plugins {
    id("com.android.library") version "8.6.1" apply false
    id("io.deepmedia.tools.deployer") version "0.16.0" apply false
    // Code formatter. Version is declared here, applied per-subproject in build.gradle.kts
    id("com.diffplug.spotless") version "8.10.3" apply false
}

rootProject.name = "ftc-ball-chase"

// One artifact per pathing generation. All three ship the same package
// (org.hivevisionftc.ballchase), so a robot project depends on exactly one of
// them. The Gradle project name is the Maven artifactId.
//
//   :ball-chase-weaver  Pedro 3 + Marrow Weaver   <- CURRENT, use this
//   :ball-chase         Pedro 3, straight-line chase   (archived, see below)
//   :ball-chase-legacy  Pedro 2                        (archived)
//
// ball-chase is kept building for teams mid-season, but new development
// (obstacle avoidance, multi-target planning, drivetrain adapters) happens in
// ball-chase-weaver, which replaces its BallChaseFollower.
include(":ball-chase-legacy")  // Pedro 2
include(":ball-chase")         // Pedro 3, archived
include(":ball-chase-weaver")  // Pedro 3 + Marrow Weaver (current)
