package org.hivevisionftc.vision;

import androidx.annotation.NonNull;
import java.util.Locale;

/**
 * Combined configuration for camera geometry and vision detection filtering.
 * <p>
 * Example usage:
 * <pre>{@code
 * VisionConfig config = new VisionConfig()
 *         .camPitchDeg(25.0)
 *         .camH(9.2)
 *         .ballH(3.0)
 *         .camXOffset(0.0)
 *         .camYOffset(0.0)
 *         .maxStalenessMs(120)
 *         .minConfidence(0.44);
 * }</pre>
 */
public class VisionConfig {
    // --- Camera Geometry ---

    /**
     * Camera tilt angle below horizontal, in degrees.
     */
    private double camPitchDeg = 0;

    /**
     * Camera lens height above floor, in inches.
     */
    private double camHeight = 0;

    /**
     * Ball center height above floor, in inches.
     */
    private double ballHeight = 3.0;

    /**
     * Camera offset forward of robot center, in inches (mount offset).
     */
    private double camXOffset = 0.0;

    /**
     * Camera offset left of robot center, in inches; negative values indicate right offset.
     */
    private double camYOffset = 0.0;

    // --- Detection Filtering ---

    /**
     * Maximum allowed staleness for detection data, in milliseconds.
     */
    private long maxStalenessMs = 120;

    /**
     * Minimum detection confidence threshold (range 0.0 to 1.0 or 0.0 to 100.0 depending on firmware).
     */
    private double minConfidence = 0.44;

    /**
     * Whether to ignore confidence when filtering detections.
     */
    private boolean ignoreConfidence = false;

    /**
     * The camera pipeline to use.
     */
    private int pipeline = 3;

    /**
     * The polling rate of the camera, in Hz.
     */
    private int pollingRateHz = 150;

    /**
     * Returns the camera tilt angle below horizontal in degrees.
     *
     * @return the camera tilt in degrees
     */
    public double getCamPitchDeg() {
        return camPitchDeg;
    }

    /**
     * Sets the camera tilt angle below horizontal in degrees.
     *
     * @param v camera tilt in degrees
     * @return this config, for chaining
     */
    public VisionConfig camPitchDeg(double v) {
        this.camPitchDeg = v;
        return this;
    }

    /**
     * Returns the camera lens height above floor in inches.
     *
     * @return the camera height in inches
     */
    public double getCamHeight() {
        return camHeight;
    }

    /**
     * Sets the camera lens height above floor in inches.
     *
     * @param v camera height in inches
     * @return this config, for chaining
     */
    public VisionConfig camHeight(double v) {
        this.camHeight = v;
        return this;
    }

    /**
     * Returns the ball center height above floor in inches.
     *
     * @return the ball height in inches
     */
    public double getBallHeight() {
        return ballHeight;
    }

    /**
     * Sets the ball center height above floor in inches.
     *
     * @param v ball height in inches
     * @return this config, for chaining
     */
    public VisionConfig ballHeight(double v) {
        this.ballHeight = v;
        return this;
    }

    /**
     * Returns the camera forward offset from robot center in inches.
     *
     * @return the camera X offset in inches
     */
    public double getCamXOffset() {
        return camXOffset;
    }

    /**
     * Sets the camera forward offset from robot center in inches.
     *
     * @param v camera X offset in inches
     * @return this config, for chaining
     */
    public VisionConfig camXOffset(double v) {
        this.camXOffset = v;
        return this;
    }

    /**
     * Returns the camera left offset from robot center in inches.
     *
     * @return the camera Y offset in inches
     */
    public double getCamYOffset() {
        return camYOffset;
    }

    /**
     * Sets the camera left offset from robot center in inches (negative for right).
     *
     * @param v camera Y offset in inches
     * @return this config, for chaining
     */
    public VisionConfig camYOffset(double v) {
        this.camYOffset = v;
        return this;
    }

    /**
     * Returns the maximum allowed staleness for detection data in milliseconds.
     *
     * @return the max staleness in milliseconds
     */
    public long getMaxStalenessMs() {
        return maxStalenessMs;
    }

    /**
     * Sets the maximum allowed staleness for detection data in milliseconds.
     *
     * @param v max staleness in milliseconds
     * @return this config, for chaining
     */
    public VisionConfig maxStalenessMs(long v) {
        this.maxStalenessMs = v;
        return this;
    }

    /**
     * Returns the minimum detection confidence threshold.
     *
     * @return the minimum confidence threshold
     */
    public double getMinConfidence() {
        return minConfidence;
    }

    /**
     * Sets the minimum detection confidence threshold.
     *
     * @param v minimum confidence threshold
     * @return this config, for chaining
     */
    public VisionConfig minConfidence(double v) {
        this.minConfidence = v;
        return this;
    }

    /**
     * Returns whether to ignore confidence when filtering detections.
     *
     * @return whether to ignore confidence
     */
    public boolean ignoreConfidence() {
        return this.ignoreConfidence;
    }

    /**
     * Sets whether to ignore confidence when filtering detections.
     *
     * @param v whether to ignore or not
     * @return this config, for chaining
     */
    public VisionConfig ignoreConfidence(boolean v) {
        this.ignoreConfidence = v;
        return this;
    }

    /**
     * Returns the camera pipeline to use.
     *
     * @return camera pipeline
     */
    public int getPipeline() {
        return this.pipeline;
    }

    /**
     * Sets the camera pipeline to use.
     *
     * @param v camera pipeline to use
     * @return this
     */
    public VisionConfig pipeline(int v) {
        this.pipeline = v;
        return this;
    }

    /**
     * Returns the polling rate of the camera, in Hz.
     * @return polling rate in hz
     */
    public int getPollingRateHz() {
        return this.pollingRateHz;
    }

    /**
     * Sets the polling rate of the camera, in Hz.
     * @param v polling rate
     * @return this, for chaining
     */
    public VisionConfig pollingRateHz(int v) {
        this.pollingRateHz = v;
        return this;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(
                Locale.ROOT,
                "VisionConfig(camPitchDeg=%.3f, camHeight=%.3f, ballHeight=%.3f, camXOffset=%.3f, camYOffset=%.3f, "
                        + "maxStalenessMs=%d, minConfidence=%.3f, ignoreConfidence=%b, pipeline=%d, pollingRateHz=%d)",
                camPitchDeg,
                camHeight,
                ballHeight,
                camXOffset,
                camYOffset,
                maxStalenessMs,
                minConfidence,
                ignoreConfidence,
                pipeline,
                pollingRateHz);
    }
}
