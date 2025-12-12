package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.teamcode.utility.LocalizationConstants.LEFT_WHEEL_POSE;
import static org.firstinspires.ftc.teamcode.utility.LocalizationConstants.RIGHT_WHEEL_POSE;
import static org.firstinspires.ftc.teamcode.utility.LocalizationConstants.encoderTicksToInches;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Two-wheel tracking localizer using parallel goBilda pods plus the IMU
 * heading.
 */
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    private final Encoder leftEncoder, rightEncoder;
    private final DoubleSupplier headingSupplier;

    public TwoWheelTrackingLocalizer(@NonNull Encoder leftEncoder,
            @NonNull Encoder rightEncoder,
            @NonNull DoubleSupplier headingSupplier) {
        super(Arrays.asList(LEFT_WHEEL_POSE, RIGHT_WHEEL_POSE));
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.headingSupplier = headingSupplier;
    }

    public TwoWheelTrackingLocalizer(@NonNull Encoder leftEncoder,
            @NonNull Encoder rightEncoder,
            @NonNull DoubleSupplier headingSupplier,
            boolean reverseLeft) {
        this(leftEncoder, rightEncoder, headingSupplier);
        if (reverseLeft) {
            this.leftEncoder.setDirection(Encoder.Direction.REVERSE);
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()));
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()));
    }

    @Override
    public double getHeading() {
        return AngleUnit.DEGREES.toRadians(headingSupplier.getAsDouble());
    }

    @Override
    public Double getHeadingVelocity() {
        // Heading velocity not used for swerve odometry, return 0.0 to keep the
        // interface happy.
        return 0.0;
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }
}
