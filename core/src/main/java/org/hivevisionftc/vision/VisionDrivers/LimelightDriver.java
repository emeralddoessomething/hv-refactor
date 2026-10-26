package org.hivevisionftc.vision.VisionDrivers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.hivevisionftc.vision.Detection;
import org.hivevisionftc.vision.Pose;
import org.hivevisionftc.vision.VisionConfig;

import java.util.ArrayList;
import java.util.List;

public class LimelightDriver {
    private final Limelight3A limelight;
    private final VisionConfig visionConfig;
    private long lastStalenessMs = Long.MAX_VALUE;

    public LimelightDriver(Limelight3A limelight, VisionConfig visionConfig) {
        this.limelight = limelight;
        this.visionConfig = visionConfig;

        int pipeline = visionConfig.getLimelightPipeline();
        int pollRate = visionConfig.getLimelightPollingRateHz();

        this.limelight.pipelineSwitch(pipeline);
        this.limelight.setPollRateHz(pollRate);

        this.limelight.start();
    }

    public ArrayList<Detection> fetchLatestDetections(Pose robotPose) {
        ArrayList<Detection> detections = new ArrayList<>();
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            lastStalenessMs = result.getStaleness();
            List<LLResultTypes.DetectorResult> limelightDetectorResult = result.getDetectorResults();

            for (LLResultTypes.DetectorResult selectedDetection : limelightDetectorResult) {
                detections.add(new Detection(robotPose, visionConfig, selectedDetection));
            }
        }

        return detections;
    }

    public ArrayList<Detection> fetchLatestDetections() {
        return fetchLatestDetections(null);
    }

    public long stalenessMs() {
        return lastStalenessMs;
    }
}
