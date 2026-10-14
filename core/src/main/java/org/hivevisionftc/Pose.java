package org.hivevisionftc;

import androidx.annotation.NonNull;

import java.util.Locale;

public class Pose {
    private final double x;
    private final double y;
    private final double headingRad;

    public Pose(double x, double y, double headingRad) {
        this.x = x;
        this.y = y;
        this.headingRad = headingRad;
    }

    public Pose(double x, double y) {
        this(x, y, 0);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeadingRad() {
        return headingRad;
    }

    @Override
    @NonNull
    public String toString() {
        return String.format(Locale.ROOT,"Pose(x=%.3f, y=%.3f, headingRad=%.3f)", x, y, headingRad);
    }
}
