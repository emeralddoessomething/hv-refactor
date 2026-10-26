package org.hivevisionftc.vision;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

public class Detection {
    private Pose pose;
    private double tx;
    private double ty;
    private double confidence;
    private double area;
    private int classId;
    private String className;

    public Detection(Pose robotPose, VisionConfig visionConfig, double tx, double ty, double confidence, double area) {
        this.tx = tx;
        this.ty = ty;
        this.confidence = confidence;
        this.area = area;
        pose = findDetectionPose(robotPose, visionConfig, tx, ty);
        // todo - class id and class name, does visionPortal probably spits out something different... enums? idk later someone(me) problem
    }

    public Detection(Pose robotPose, VisionConfig visionConfig, LLResultTypes.DetectorResult detectorResult) {
        tx = detectorResult.getTargetXDegrees();
        ty = detectorResult.getTargetYDegrees();
        confidence = detectorResult.getConfidence();
        area = detectorResult.getTargetArea();
        pose = findDetectionPose(robotPose, visionConfig, detectorResult);
        classId = detectorResult.getClassId();
        className = detectorResult.getClassName();
    }



    public Pose getPose() {
        return pose;
    }

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public double getTx() {
        return tx;
    }

    public void setTx(double tx) {
        this.tx = tx;
    }

    public double getTy() {
        return ty;
    }

    public void setTy(double ty) {
        this.ty = ty;
    }

    public double getConfidence() {
        return confidence;
    }

    public void setConfidence(double confidence) {
        this.confidence = confidence;
    }

    public double getArea() {
        return area;
    }

    public void setArea(double area) {
        this.area = area;
    }

    public int getClassId() {
        return classId;
    }

    public void setClassId(int classId) {
        this.classId = classId;
    }

    public String getClassName() {
        return className;
    }

    public void setClassName(String className) {
        this.className = className;
    }
}
