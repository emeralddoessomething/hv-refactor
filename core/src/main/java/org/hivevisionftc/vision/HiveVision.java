package org.hivevisionftc.vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.hivevisionftc.vision.VisionDrivers.LimelightDriver;

import java.util.ArrayList;

public class HiveVision {
    Limelight3A limelight;
    VisionConfig visionConfig;
    LimelightDriver limelightDriver;

    public HiveVision(Limelight3A limelight, VisionConfig visionConfig) {
        this.limelight = limelight;
        this.visionConfig = visionConfig;
        limelightDriver = new LimelightDriver(limelight, visionConfig);
    }

    public ArrayList<Detection> fetchLimelightDetections(Pose robotPose) {
        return limelightDriver.fetchLatestDetections(robotPose);
    }
}
