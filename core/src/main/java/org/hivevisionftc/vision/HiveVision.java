package org.hivevisionftc.vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Set;


public class HiveVision {
    private final DetectionSource source;
    private final VisionConfig config;
    private Set<BiobuzzClasses> allowedClasses;

    public HiveVision(DetectionSource source, VisionConfig config) {
        this.source = source;
        this.config = config;
        this.allowedClasses = Set.of(BiobuzzClasses.POLLEN, BiobuzzClasses.NECTAR_RED, BiobuzzClasses.NECTAR_BLUE);
    }

    public HiveVision(Limelight3A limelight, VisionConfig config) {
        this(new LimelightSource(limelight, config.getPipeline()), config);
    }

    public List<Detection> getRawDetections() {
        return source.latest();
    }

    public long getStalenessMs() {
        return source.stalenessMs();
    }

    public HiveVision setAllowedClasses(BiobuzzClasses... classes) {
        allowedClasses.clear();
        allowedClasses = Set.of(classes);
        return this;
    }

    public HiveVision addAllowedClass(BiobuzzClasses clazz) {
        allowedClasses.add(clazz);
        return this;
    }

    public boolean isAllowed(BiobuzzClasses clazz) {
        return allowedClasses.contains(clazz);
    }

    public Set<BiobuzzClasses> getAllowed() {
        return allowedClasses;
    }

    public class DetectionList {
        private final List<Detection> detectionList;
        private Pose robotPose;
        private DetectionList() {
            detectionList = new ArrayList<>();
        }

        public DetectionList fetch(double robotX, double robotY) {
            List<Detection> detections = source.latest();
            if (detections == null) return this;
            if (!detections.isEmpty()) detectionList.clear();
            robotPose = new Pose(robotX, robotY);

            for (Detection detection : detections) {
                if (!allowedClasses.contains(detection.getBallClass())) continue; //Skip disallowed colors
                if (!config.ignoreConfidence() && detection.getConfidence() < config.getMinConfidence()) continue; // Skip low confidence detections

                detectionList.add(getAbsolutePosition(detection, robotPose));
            }

           return this;
        }

        public DetectionList fetch(Pose robotPose) {
            return fetch(robotPose.getX(), robotPose.getY());
        }
        // ─── Sorting ─────────────────────────────────────────────────────────────

        /** Sorts by straight-line distance from the robot's pose (closest first). */
        public DetectionList sortByDistance() {
            detectionList.sort(Comparator.comparingDouble(a ->
                    Math.hypot(
                            a.getX() - robotPose.getX(),
                            a.getY() - robotPose.getY()
                    )
            ));

            return this;
        }

        /** Sorts by color ordinal ascending (POLLEN, NECTAR_RED, NECTAR_BLUE, UNKNOWN). */
        public DetectionList sortByColor() {
            detectionList.sort(Comparator.comparingInt(i -> i.getBallClass().getId()));
            return this;
        }

        /** Sorts by area descending (largest first). */
        public DetectionList sortBySize() {
            detectionList.sort(Comparator.comparingDouble(Detection::getArea).reversed());
            return this;
        }

        public DetectionList sortByConfidence() {
            detectionList.sort(Comparator.comparingDouble(Detection::getConfidence).reversed());
            return this;
        }

        // ─── Filtering ───────────────────────────────────────────────────────────
        public DetectionList clampYDetections(double minY, double maxY) {
            detectionList.removeIf(d -> d.getY() < minY);
            detectionList.removeIf(d -> d.getY() > maxY);
            return this;
        }

        // ─── Terminals ───────────────────────────────────────────────────────────

        /** Returns the closest artifact, or {@code null} if the list is empty. */
        public Detection getClosest() {
            if (detectionList.isEmpty()) return null;
            return sortByDistance().detectionList.get(0);
        }

        public Detection getBiggest() {
            if (detectionList.isEmpty()) return null;
            return sortBySize().detectionList.get(detectionList.size() - 1);
        }

        /** Returns a copy of the current artifact list. */
        public List<Detection> getDetections() {
            return new ArrayList<>(detectionList);
        }

        public int count()      { return detectionList.size(); }
        public boolean didDetect() { return !detectionList.isEmpty(); }


        private double getDistance(double ty) {
            double angleGoalRag = Math.toRadians(config.getCamPitchDeg() + ty);
            return (config.getBallHeight() - config.getCamHeight()) / Math.tan(angleGoalRag);
        }

        private Pose getRelativePose(double distance, double tx) {
            double theta = Math.toRadians(tx);

            double deltaX = distance * Math.cos(theta) + config.getCamXOffset();
            double deltaY = -distance * Math.sin(theta) + config.getCamYOffset();

            return new Pose(deltaX, deltaY);
        }

        private Pose getAbsolutePosition(Pose relativePose, Pose robotPose) {
            double theta = robotPose.getHeadingRad();
            double x = relativePose.getX() * Math.cos(theta) - relativePose.getY() * Math.sin(theta);
            double y = relativePose.getX() * Math.sin(theta) + relativePose.getY() * Math.cos(theta);


            return new Pose(
                    robotPose.getX() + x,
                    robotPose.getY() + y,
                    0);
        }

        private Detection getAbsolutePosition(Detection detection, Pose robotPose) {
            double distance = getDistance(detection.getY());
            Pose pose = getAbsolutePosition(getRelativePose(distance, detection.getX()), robotPose);
            return new Detection(detection, pose);
        }
    }
}
