package org.hivevisionftc.vision;

/**
 * Vision configuration, which includes camera geometry and a smattering of other settings (min confidence, max staleness).
 */
public class VisionConfig {
    /**
     * Pitch of the camera lens, in degrees, from the horizontal horizon (parallel to the ground).
     * <p>
     * If your camera is pitched up, you will input a positive number. If your camera is pitched down, you will input a negative number.
     */
    private double cameraPitchDeg = 0;

    /**
     * Height of the camera lens, in inches, from the ground.
     * <p>
     * If your camera is... above the ground... this number will be positive. If your camera is underground (???) this number will be negative.
     */
    private double cameraHeightIn = 0;

    /**
     * Offset of the camera lens, in inches, from the center of the robot (forwards and backwards).
     * <p>
     * If your camera is mounted on the front of the bot, this number will be positive. If your camera is mounted on the rear of the bot, this number will be negative.
     */
    private double cameraForwardIn = 0;

    /**
     * Offset of the camera lens, in inches, from the center of the robot (left and right)
     * <p>
     * If your camera is mounted left from the center of the bot, this number will be positive. If your camera is mounted to the right of the center of the bot, this number will be negative.
     */
    private double cameraLeftIn = 0;

    /**
     * The max staleness of the detections, in milliseconds.
     * <p>
     * 'staleness' represents the amount of time from the last time vision updated its detections. This means that if the last time vision processed an image was 100ms ago, then 'staleness', or how stale a detection is, would me 100ms.
     */
    private double maxStalenessMs = 1000;

    /**
     * The minimum amount of confidence a detection needs to have to be classified. This WILL BE IGNORED for limelights.
     * <p>
     * HEY! If you're using a limelight, you may want to check the confidence threshold you have set in the limelight webpage. If its too low, then the data sent to process (on the control hub) may not see a detection because the limelight is omitting it, even if you have the min confidence in here set to something really low, like zero.
     * <p>
     * 'confidence' is a decimal from [0,1] that represents how confident the neural model is that what it's detecting is what it thinks it is. Say its trying to detect a ball, if it is absolutely sure what it sees is a ball, confidence will be very high (eg. 0.90), while if it's not so sure, it may be lower (eg. 0.20)
     */
    private double minConfidence = 0.5;

    /**
     * The pipeline to use, from [0,7].
     * <p>
     * HEY! This applies to Limelights only!
     */
    private int limelightPipeline = 0;

    /**
     * The polling rate, in Hz (or 1/s), from [1,250].
     * <p>
     * HEY! This applies to limelights only!
     * <p>
     * 'polling rate' refers to how often the control hub requests new data from the limelight.
     */
    private int limelightPollingRateHz = 100;

    // getter setters
    public double getCameraPitchDeg() {
        return cameraPitchDeg;
    }

    public void setCameraPitchDeg(double cameraPitchDeg) {
        this.cameraPitchDeg = cameraPitchDeg;
    }

    public double getCameraHeightIn() {
        return cameraHeightIn;
    }

    public void setCameraHeightIn(double cameraHeightIn) {
        this.cameraHeightIn = cameraHeightIn;
    }

    public double getCameraForwardIn() {
        return cameraForwardIn;
    }

    public void setCameraForwardIn(double cameraForwardIn) {
        this.cameraForwardIn = cameraForwardIn;
    }

    public double getCameraLeftIn() {
        return cameraLeftIn;
    }

    public void setCameraLeftIn(double cameraLeftIn) {
        this.cameraLeftIn = cameraLeftIn;
    }

    public double getMaxStalenessMs() {
        return maxStalenessMs;
    }

    public void setMaxStalenessMs(double maxStalenessMs) {
        this.maxStalenessMs = maxStalenessMs;
    }

    public double getMinConfidence() {
        return minConfidence;
    }

    public void setMinConfidence(double minConfidence) {
        this.minConfidence = minConfidence;
    }

    public int getLimelightPipeline() {
        return limelightPipeline;
    }

    public void setLimelightPipeline(int limelightPipeline) {
        this.limelightPipeline = limelightPipeline;
    }

    public int getLimelightPollingRateHz() {
        return limelightPollingRateHz;
    }

    public void setLimelightPollingRateHz(int limelightPollingRateHz) {
        this.limelightPollingRateHz = limelightPollingRateHz;
    }

    // builder
    public VisionConfig cameraPitchDeg(double cameraPitchDeg) {
        this.cameraPitchDeg = cameraPitchDeg;
        return this;
    }

    public VisionConfig cameraHeightIn(double cameraHeightIn) {
        this.cameraHeightIn = cameraHeightIn;
        return this;
    }

    public VisionConfig cameraForwardIn(double cameraForwardIn) {
        this.cameraForwardIn = cameraForwardIn;
        return this;
    }

    public VisionConfig cameraLeftIn(double cameraLeftIn) {
        this.cameraLeftIn = cameraLeftIn;
        return this;
    }

    public VisionConfig maxStalenessMs(double maxStalenessMs) {
        this.maxStalenessMs = maxStalenessMs;
        return this;
    }

    public VisionConfig minConfidence(double minConfidence) {
        this.minConfidence = minConfidence;
        return this;
    }

    public VisionConfig limelightPipeline(int limelightPipeline) {
        this.limelightPipeline = limelightPipeline;
        return this;
    }

    public VisionConfig limelightPollingRateHz(int limelightPollingRateHz) {
        this.limelightPollingRateHz = limelightPollingRateHz;
        return this;
    }
}
