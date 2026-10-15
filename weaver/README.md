# HiveVision Weaver

Optional module: plans a [Marrow Weaver](https://marrow.skeletonarmyftc.com/automations/weaver) route
through the balls in a `HiveVision.DetectionList`.

```gradle
implementation 'org.hivevisionftc:weaver:1.0.0'   // pulls in :core and Marrow
```


Example usage:
```java
PathResult path = vision.detections()
        .fetch(follower.getPose().getX(), follower.getPose().getY())
        .with(WeaverRoute.fromFetchPose()
                .avoid(new CircleZone(new Point(72, 72), 18))   // optional obstacles
                .maxTargets(4));                                 // optional cap

if (path != null) follower.follow(PedroPathingConverter.toPath(path));
```

- `with(...)` is the core's extension point; the module adds no methods to `DetectionList`, so core never
  depends on Marrow.
- Returns `null` when nothing was detected.
- Detections must be in field coordinates, which is what `fetch(...)` produces.
