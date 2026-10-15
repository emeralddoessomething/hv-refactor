plugins {
    base // provides the clean task
}

subprojects {
    apply(plugin = "com.android.library")
    apply(plugin = "io.deepmedia.tools.deployer")
    apply(plugin = "com.diffplug.spotless")

    group = "org.hivevisionftc"
    version = "1.0.0"

    repositories {
        mavenCentral()
        google()
        maven("https://maven.brott.dev")
    }

    // Spotless: run `./gradlew spotlessApply` to format, `./gradlew spotlessCheck` to verify
    // (spotlessCheck also runs as part of `check`)
    extensions.configure<com.diffplug.gradle.spotless.SpotlessExtension> {
        java {
            // Explicit target, since Android source sets aren't picked up automatically
            target("src/**/*.java")
            palantirJavaFormat()
            removeUnusedImports()
            shortenFullyQualifiedTypes()
            importOrder()
            trimTrailingWhitespace()
        }
    }

    extensions.configure<io.deepmedia.tools.deployer.DeployerExtension> {
        projectInfo {
            name.set("HiveVision")
            description.set("Camera based detection and positioning library for FTC.")
            url.set("https://github.com/hivevisionftc/Hive-Vision.git")
            scm.fromGithub("hivevisionftc", "Hive-Vison")
            developer("Hive Vision", "hivevisionftc@gmail.com")
            license(MIT)
        }

        signing {
            key.set(secret("SIGNING_KEY"))
            password.set(secret("SIGNING_PASSWORD"))
        }

        nexusSpec("snapshot") {
            repositoryUrl.set("https://central.sonatype.com/repository/maven-snapshots/")

            auth {
                user.set(secret("CENTRAL_PORTAL_USER"))
                password.set(secret("CENTRAL_PORTAL_PASSWORD"))
            }
        }

        centralPortalSpec {
            auth {
                user.set(secret("CENTRAL_PORTAL_USER"))
                password.set(secret("CENTRAL_PORTAL_PASSWORD"))
            }
        }

        localSpec() // For publishing to Maven Local

        content {
            androidComponents("release")
        }
    }
}

val publishedSubprojects = subprojects.toList()

tasks.named<Delete>("clean") {
    delete(rootProject.layout.buildDirectory)
    delete(subprojects.map { it.layout.buildDirectory })
}

tasks.register("deployCentralPortal") {
    group = "publishing"
    description = "Publishes all subprojects to Maven Central."
    dependsOn("clean")
    dependsOn(publishedSubprojects.map { it.tasks.named("deployCentralPortal") })
}

tasks.register("deployNexusSnapshot") {
    group = "publishing"
    description = "Publishes all subprojects to Maven Central Snapshots."
    dependsOn("clean")
    dependsOn(publishedSubprojects.map { it.tasks.named("deployNexusSnapshot") })
}

tasks.register("deployLocal") {
    group = "publishing"
    description = "Publishes all subprojects to Maven Local."
    dependsOn(publishedSubprojects.map { it.tasks.named("deployLocal") })
}

tasks.register("spotlessCheck") {
    group = "spotless"
    description = "Runs spotless check on all modules which apply the spotless plugin."
    dependsOn(publishedSubprojects.map { it.tasks.named("spotlessCheck") })
}

tasks.register("spotlessApply") {
    group = "spotless"
    description = "Runs spotless apply on all modules which apply the spotless plugin."
    dependsOn(publishedSubprojects.map { it.tasks.named("spotlessApply") })
}
