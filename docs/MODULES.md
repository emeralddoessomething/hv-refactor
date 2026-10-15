# Module Reference — HiveVision

This document provides a comprehensive API and architecture reference for the `HiveVision` library.

---

## Overview

`HiveVision` is architected as a modular library:
- **`:core` (`org.hivevisionftc.vision`)**: Perception pipeline, Limelight 3A abstraction, configuration, detection filtering, sorting, and 3D camera-to-field projection.
- **`:weaver` (`org.hivevisionftc.vision.weaver`)**: Extension module that integrates with [Marrow Weaver](https://marrow.skeletonarmyftc.com/) to plan bezier path routes through field detections.

---

## Core Module (`org.hivevisionftc.vision`)

### `HiveVision`

The main entry point for managing vision pipeline sessions. It ties together a camera hardware source (`DetectionSource`), configuration parameters (`VisionConfig`), and allowed detection classes.

#### Constructors
```java
public HiveVision(DetectionSource source, VisionConfig config);
public HiveVision(Limelight3A limelight, VisionConfig config);
```

#### Methods

| Method                                                    | Description                                                                                 |
|-----------------------------------------------------------|---------------------------------------------------------------------------------------------|
| `HiveVision setAllowedClasses(BiobuzzClasses... classes)` | Sets the list of allowed target classes to filter for (overwrites current allowed classes). |
| `HiveVision addAllowedClass(BiobuzzClasses clazz)`        | Adds a specific class to the allowed class set.                                             |
| `boolean isAllowed(BiobuzzClasses clazz)`                 | Checks if a class is currently allowed.                                                     |
| `Set<BiobuzzClasses> getAllowed()`                        | Returns the active set of allowed classes.                                                  |
| `List<Detection> getRawDetections()`                      | Retrieves the raw detection list directly from the underlying detection source.             |
| `long getStalenessMs()`                                   | Returns data staleness in milliseconds from the detection source.                           |
| `DetectionList detections()`                              | Creates a new `DetectionList` bound to this vision instance for fetching and processing.    |

---

### `HiveVision.DetectionList`

A fluent builder and query container for fetching, filtering, sorting, and transforming detection results into absolute field coordinates.

#### Fetching & Projection
```java
public DetectionList fetch(double robotX, double robotY);
public DetectionList fetch(FieldPose robotPose);
```
Queries the `DetectionSource` for the latest detections, filters them by allowed classes, minimum confidence threshold (`minConfidence`), and data staleness (`maxStalenessMs`), then projects camera-relative angles (`tx`, `ty`) into field coordinates (`x`, `y` in inches) based on camera mount geometry and the given robot pose.

#### Sorting Methods
- **`sortByDistance()`**: Sorts detections by straight-line distance from the robot pose (closest first).
- **`sortByColor()`**: Sorts detections by class ID ordinal ascending (`POLLEN`, `NECTAR_RED`, `NECTAR_BLUE`, `UNKNOWN`).
- **`sortBySize()`**: Sorts detections by target area descending (largest first).
- **`sortByConfidence()`**: Sorts detections by confidence score descending.

#### Filtering Methods
- **`clampYDetections(double minY, double maxY)`**: Removes detections whose field Y-coordinate falls outside `[minY, maxY]`.

#### Terminal & Query Methods
- **`Detection getClosest()`**: Returns the closest detection relative to robot pose, or `null` if empty.
- **`Detection getBiggest()`**: Returns the detection with the largest bounding area, or `null` if empty.
- **`List<Detection> getDetections()`**: Returns a copy of the list of projected field detections.
- **`int count()`**: Returns the number of valid detections in the list.
- **`boolean didDetect()`**: Returns `true` if one or more valid detections exist.
- **`FieldPose getRobotPose()`**: Returns the robot pose used during the last `fetch(...)` call.

#### Extension Point
```java
public <R> R with(Function<? super DetectionList, ? extends R> fn);
```
Passes this `DetectionList` instance into the provided function and returns its result. This allows optional external modules (such as `:weaver`) to operate on detection lists without creating direct dependencies in `:core`.

Example:
```java
PathResult path = vision.detections()
        .fetch(robotPose)
        .sortByDistance()
        .with(WeaverRoute.fromFetchPose());
```

---

### `VisionConfig`

A fluent builder class that holds camera mounting geometry parameters, detection threshold filters, and hardware polling options.

```java
VisionConfig config = new VisionConfig()
        .camPitchDeg(25.0)      // Downward tilt angle below horizontal (degrees)
        .camHeight(9.2)         // Lens height above floor (inches)
        .ballHeight(3.0)        // Game element center height above floor (inches)
        .camXOffset(4.0)        // Forward offset from robot center (inches)
        .camYOffset(0.0)        // Left offset from robot center (inches, negative for right)
        .maxStalenessMs(120)    // Max allowed detection staleness (milliseconds)
        .minConfidence(0.44)    // Minimum confidence score threshold (0.0 - 1.0)
        .ignoreConfidence(false)// If true, skips confidence filtering
        .pipeline(3)            // Camera pipeline index
        .pollingRateHz(150);    // Limelight polling rate in Hz
```

---

### `Detection`

Represents a single detected target. Before `fetch(...)`, `x` and `y` represent raw camera angles (`tx`, `ty` in degrees). After `fetch(...)`, `x` and `y` represent projected field coordinates in inches.

#### Getters
- `BiobuzzClasses getBallClass()`: Target classification.
- `double getX()`: Target X position (`tx` degrees or field X inches).
- `double getY()`: Target Y position (`ty` degrees or field Y inches).
- `double getConfidence()`: Detection confidence score.
- `double getArea()`: Detection bounding box area.

---

### `FieldPose`

An immutable 2D pose representing location and heading on the field.

```java
public FieldPose(double x, double y, double heading);
public FieldPose(double x, double y); // heading defaults to 0.0
```

- `double getX()`: X position in field inches.
- `double getY()`: Y position in field inches.
- `double getHeading()`: Robot orientation heading in radians.

---

### `BiobuzzClasses`

Enum representing game element classifications:
- `POLLEN(0)` — Neutral yellow pollen element
- `NECTAR_RED(1)` — Alliance red nectar element
- `NECTAR_BLUE(2)` — Alliance blue nectar element
- `UNKNOWN(Integer.MAX_VALUE)` — Fallback safety classification

Includes `getId()` and static `BiobuzzClasses.fromId(int id)`.

---

### `DetectionSource` & `LimelightSource`

`DetectionSource` is the interface for vision backends:
```java
public interface DetectionSource {
    List<Detection> latest();
    long stalenessMs();
}
```

`LimelightSource` implements `DetectionSource` for the FTC `Limelight3A` camera, managing pipeline switches, polling rate, and mapping `LLResult` detector outputs into `Detection` instances.

---

## Weaver Module (`org.hivevisionftc.vision.weaver`)

### `WeaverRoute`

Implements `Function<HiveVision.DetectionList, PathResult>`. It uses [Marrow Weaver](https://marrow.skeletonarmyftc.com/) to build cubic bezier routes passing through field-frame detections.

#### Factory Methods
- **`WeaverRoute.from(FieldPose start)`**: Specifies an explicit starting pose for the path route.
- **`WeaverRoute.fromFetchPose()`**: Uses the robot pose passed during the previous `DetectionList.fetch(...)` call.

#### Configuration Methods
- **`avoid(Zone... zones)` / `addObstacle(Zone zone)` / `avoid(Collection<? extends Zone> zones)`**: Adds obstacle zones (such as `CircleZone`) that the generated path must route around.
- **`maxTargets(int n)`**: Sets the maximum number of targets to include in a single path route (`0` or negative means no limit).
- **`ordered()`**: Configures Weaver to visit targets in the order they appear in the `DetectionList` (bypassing cost-ordering).
- **`end(FieldPose end)`**: Sets an explicit destination pose for the end of the route.

#### Execution
```java
PathResult path = WeaverRoute.from(startPose)
        .avoid(new CircleZone(new Point(72, 72), 18.0))
        .maxTargets(3)
        .plan(fieldDetections, startPose);
```
Or via the fluent `DetectionList.with(...)` chain:
```java
PathResult path = vision.detections()
        .fetch(robotPose)
        .sortByDistance()
        .with(WeaverRoute.fromFetchPose().maxTargets(3));
```
Returns a `PathResult` object (or `null` if no valid detections exist).
