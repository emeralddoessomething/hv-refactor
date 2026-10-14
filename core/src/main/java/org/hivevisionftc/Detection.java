package org.hivevisionftc;

public class Detection {
    private final BiobuzzClasses ballClass;
    private final double x, y, confidence, area;

    public Detection(int classId, double txDeg, double tyDeg, double confidence, double area) {
        this.ballClass = BiobuzzClasses.fromId(classId);
        this.x = txDeg;
        this.y = tyDeg;
        this.confidence = confidence;
        this.area = area;
    }

    public Detection(Detection detection) {
        this.ballClass = detection.ballClass;
        this.x = detection.x;
        this.y = detection.y;
        this.confidence = detection.confidence;
        this.area = detection.area;
    }

    public Detection(Detection detection, Pose newCords) {
        this.ballClass = detection.ballClass;
        this.x = newCords.getX();
        this.y = newCords.getX();
        this.confidence = detection.confidence;
        this.area = detection.area;
    }

    public BiobuzzClasses getBallClass() {
        return ballClass;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getConfidence() {
        return confidence;
    }

    public double getArea() {
        return area;
    }

}
