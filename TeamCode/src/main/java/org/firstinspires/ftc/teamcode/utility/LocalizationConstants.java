package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Tunable constants for the two-wheel odometry pods. Values here are a
 * reasonable baseline for goBilda dead wheels but should be characterized on
 * your robot.
 */
public final class LocalizationConstants {
    /** Encoder ticks per full rotation for the goBilda encoder. */
    public static final double TICKS_PER_REV = 8192;
    /** Wheel radius in inches (goBilda 2.75" odometry wheel is ~1.377" radius). */
    public static final double WHEEL_RADIUS = 1.377;
    /** Output/input gear ratio. Set to 1.0 if the encoder is mounted directly to the wheel. */
    public static final double GEAR_RATIO = 1.0;

    /** Distance between the left and right tracking wheels. */
    public static final double TRACK_WIDTH = 10.5; // in
    /** Forward offset of the parallel (left/right) wheel from the robot center. */
    public static final double FORWARD_OFFSET = 0.0; // in

    /**
     * Pose of the left tracking wheel relative to the robot center.
     * X is forward; Y is left.
     */
    public static final Pose2d LEFT_WHEEL_POSE = new Pose2d(FORWARD_OFFSET, TRACK_WIDTH / 2.0, 0);
    /** Pose of the right tracking wheel relative to the robot center. */
    public static final Pose2d RIGHT_WHEEL_POSE = new Pose2d(FORWARD_OFFSET, -TRACK_WIDTH / 2.0, 0);

    private LocalizationConstants() {
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
