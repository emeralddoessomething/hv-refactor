# Calibration — Measuring Camera Parameters and Coordinates

Accurate distance estimation and field-to-robot coordinate transforms depend on correct physical measurement of camera geometry and configuration parameters in `VisionConfig`.

---

## Configuration Parameters (`VisionConfig`)

| Method | Description | Units |
|--------|-------------|-------|
| `camHeight(double)` | Distance from floor to center of camera lens | Inches |
| `ballHeight(double)` | Distance from floor to center of target game element | Inches |
| `camPitchDeg(double)` | Camera optic axis tilt angle **below horizontal** | Degrees |
| `camXOffset(double)` | Camera lens position **forward** of robot reference center | Inches |
| `camYOffset(double)` | Camera lens position **left** of robot reference center (+ left, − right) | Inches |
| `minConfidence(double)` | Minimum detection confidence score to accept | 0.0 – 1.0 |
| `maxStalenessMs(long)` | Maximum allowed detection data age | Milliseconds |

---

## Physical Measurement Procedure

> [!TIP]
> **Use CAD Measurements When Available:** If your robot is modeled in CAD (such as Onshape, Fusion 360, or SOLIDWORKS), pull `camHeight`, `camPitchDeg`, `camXOffset`, and `camYOffset` directly from your CAD assembly. CAD measurements are typically significantly more accurate than manual tape/ruler measurements. Be sure to double-check against the physical robot to account for tile compression, sagging, or mounting tolerances.

### 1. Camera Height (`camHeight`)
Measure vertically from the **floor/field mat to the center of the camera lens**.

```text
       CAMERA
          ●     ← Measure to lens center
          │
          │ camHeight
──────────┴────────── Floor / Mat
```

### 2. Ball Height (`ballHeight`)
Measure vertically from the **floor/field mat to the center of the target element**.

```text
       ●
      /_\       ← Center of game element (half of total height)
      │ ballHeight
─────────── Floor / Mat
```

### 3. Camera Pitch Angle (`camPitchDeg`)
Measure the downward tilt angle of the camera optic axis below horizontal. Place a digital protractor or angle gauge against the side of the camera housing.

```text
          Camera
            ●
             \
              \  25°   ← camPitchDeg (tilt below horizontal)
               \
───────────────────────── Horizontal
```

### 4. Camera Mount Offsets (`camXOffset`, `camYOffset`)
Measure from the robot's physical center (drivetrain center of rotation) to the camera lens center:

```text
                 FRONT (+X)
                   ↑
             Camera ●
                   │  camXOffset (forward, +)
                   │
             Robot ● Center
```

- **`camXOffset`**: Forward distance from robot center to camera lens (+ forward, − backward).
- **`camYOffset`**: Left/right distance from robot center (+ left, − right).

---

## Coordinate Systems and Conventions

`HiveVision` works across three reference frames:

### 1. Robot Reference Frame
```text
            +Y (Strafe LEFT)
              ↑
              │
 -X ←--------- ROBOT ----------→ +X (Forward)
              │
              ↓
            -Y (Strafe RIGHT)
```

### 2. Limelight Camera Frame (`tx`, `ty`)
```text
      tx < 0           tx = 0           tx > 0
        \                |                /
         \               |               /
          \              |              /
           ●─────────────●─────────────●   Camera Lens

   + ty = UP,  − ty = DOWN
```

When `tx > 0`, the target is to the **right** of camera center.

### 3. Field Reference Frame (Pedro Pathing Compatible)
```text
            +Y Field
              ↑
              │
 ─────────────┼─────────────→ +X Field
            Robot
```

`DetectionList.fetch(...)` converts camera-relative angles (`tx`, `ty`) into 3D range and projects them into this 2D Field Reference Frame using the robot's pose `(x, y, heading)`.

---

## Code Example

```java
VisionConfig config = new VisionConfig()
        .camPitchDeg(25.0)      // 25 degrees tilt below horizontal
        .camHeight(9.25)        // Camera lens is 9.25" above floor
        .ballHeight(3.0)        // Element center is 3.0" above floor
        .camXOffset(4.5)        // Camera mounted 4.5" forward of robot center
        .camYOffset(-1.2)       // Camera mounted 1.2" to the right of robot center
        .minConfidence(0.40)    // 40% minimum confidence threshold
        .maxStalenessMs(120)    // Discard data older than 120ms
        .pipeline(3)            // Use Limelight pipeline 3
        .pollingRateHz(150);    // Poll at 150 Hz
```
