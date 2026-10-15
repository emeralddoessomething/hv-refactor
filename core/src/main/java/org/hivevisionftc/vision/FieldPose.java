package org.hivevisionftc.vision;

public class FieldPose {
    private final double x;
    private final double y;
    private final double heading;

    public FieldPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public FieldPose(double x, double y) {
        this(x, y, 0);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }
}
