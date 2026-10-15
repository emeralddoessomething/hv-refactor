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

rootProject.name = "HiveVision"


include(":core")  // Base core module, required for all other modules
include(":weaver")  // Optional: Marrow Weaver path generation for DetectionList
