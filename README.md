# HiveVision

Camera-based detection, target filtering, field positioning, and Weaver path routing library for FTC robots.

`HiveVision` turns Limelight 3A neural network detections into absolute field coordinates and generates optimal path routes for autonomous navigation. It is built natively for FTC game elements (such as `POLLEN`, `NECTAR_RED`, `NECTAR_BLUE`) using Limelight 3A SSD detectors and integrates seamlessly with [Pedro Pathing](https://github.com/pedropathing/core) and [Marrow Weaver](https://marrow.skeletonarmyftc.com/).

---

## Modules

`HiveVision` is split into two lightweight artifacts:

| Artifact | Description | Dependencies |
|----------|-------------|--------------|
| `org.hivevisionftc:core` | Perception pipeline, Limelight 3A integration, detection filtering, sorting, and camera-to-field coordinate projection. | FTC SDK (`RobotCore`, `Hardware`) |
| `org.hivevisionftc:weaver` | Optional path routing module that converts detection lists into obstacle-avoiding [Marrow Weaver](https://marrow.skeletonarmyftc.com/) bezier paths. | `:core`, `Marrow core` |

---

## Installation

Add the required dependencies to your `TeamCode/build.gradle`:

```gradle
dependencies {
    // Core vision module
    implementation 'org.hivevisionftc:core:1.0.0'

    // Optional: Marrow Weaver route planning
    implementation 'org.hivevisionftc:weaver:1.0.0'
}
```

The artifacts publish to Maven Central. Ensure `mavenCentral()` is included in your `repositories` block.

---

## Quickstart

### 1. Configure and Initialize Vision

```java
// Define camera geometry and detection thresholds
VisionConfig config = new VisionConfig()
        .camPitchDeg(25.0)          // Camera tilt angle below horizontal
        .camHeight(9.2)             // Camera lens height above floor (inches)
        .ballHeight(3.0)            // Game element center height above floor (inches)
        .camXOffset(4.0)            // Camera forward offset from robot center (inches)
        .camYOffset(0.0)            // Camera left offset from robot center (inches)
        .minConfidence(0.44)        // Minimum detection confidence threshold
        .maxStalenessMs(120)        // Max allowed data staleness (ms)
        .pipeline(3)                // Limelight pipeline index
        .pollingRateHz(150);        // Camera polling rate

// Initialize Limelight source & vision session
Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
HiveVision vision = new HiveVision(limelight, config);

// Optionally restrict allowed detection classes
vision.setAllowedClasses(BiobuzzClasses.NECTAR_RED, BiobuzzClasses.NECTAR_BLUE);
```

### 2. Fetch and Query Detections

```java
// Fetch detections and project them to field coordinates given robot's current pose
FieldPose robotPose = new FieldPose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());

// Query the closest detected element
Detection target = vision.detections()
        .fetch(robotPose)
        .sortByDistance()
        .getClosest();

if (target != null) {
    telemetry.addData("Target Class", target.getBallClass());
    telemetry.addData("Target Field X", target.getX());
    telemetry.addData("Target Field Y", target.getY());
}
```

### 3. Generate Weaver Bezier Paths (Auto Routing)

```java
// Convert detected elements directly into an obstacle-avoiding Weaver path result
PathResult path = vision.detections()
        .fetch(robotPose)
        .sortByDistance()
        .with(WeaverRoute.fromFetchPose()
                .avoid(new CircleZone(new Point(72, 72), 18.0)) // Add obstacle zones
                .maxTargets(4));                                // Limit path targets

if (path != null) {
    follower.follow(PedroPathingConverter.toPath(path));
}
```

---

## Calibration

Accurate distance estimation and field projection require proper physical measurement of your camera mounting geometry. Refer to [docs/calibration.md](docs/calibration.md) for step-by-step measurement procedures and coordinate system conventions.

| Config Method | Description |
|---------------|-------------|
| `camPitchDeg(double)` | Downward tilt angle of the camera optic axis in degrees below horizontal. |
| `camHeight(double)` | Distance from floor to center of camera lens (inches). |
| `ballHeight(double)` | Distance from floor to center of target game element (inches). |
| `camXOffset(double)` | Forward (+X) offset of camera lens relative to robot center (inches). |
| `camYOffset(double)` | Left (+Y) offset of camera lens relative to robot center (inches, negative for right). |
| `minConfidence(double)` | Confidence threshold for accepting detections (0.0 to 1.0). |
| `maxStalenessMs(long)` | Maximum time in milliseconds before detection data is discarded. |

---

## Documentation & Examples

- **[MODULES.md](docs/MODULES.md)** — Complete API and class reference for `:core` and `:weaver`.
- **[calibration.md](docs/calibration.md)** — Procedure for measuring camera offsets, pitch angles, and understanding coordinate frames.
- **[weaver/README.md](weaver/README.md)** — Quick guide to using the `WeaverRoute` extension.
- **[docs/examples/](docs/examples/)** — Sample OpModes demonstrating integration with Pedro Pathing.

---

## Building and Testing

```bash
# Run unit test suites (JVM-based, no camera required)
./gradlew test

# Format code according to Spotless Java rules
./gradlew spotlessApply

# Check code formatting
./gradlew spotlessCheck

# Build release library archives (AARs, sources, javadocs)
./gradlew assembleRelease
```

---

## Publishing to Maven Central

Publishing uses the Deployer plugin configured in `build.gradle.kts`.

| Gradle Task | Description |
|-------------|-------------|
| `./gradlew deployLocal` | Publishes artifacts to Maven Local (`~/.m2`) for testing. |
| `./gradlew deployNexusSnapshot` | Publishes snapshot builds to Sonatype Nexus. |
| `./gradlew deployCentralPortal` | Publishes release builds to Maven Central. |

---

## License

MIT License. See [LICENSE](LICENSE) for details.
