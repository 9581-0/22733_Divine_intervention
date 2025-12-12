package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.utility.Encoder;
import org.firstinspires.ftc.teamcode.utility.TwoWheelTrackingLocalizer;

import java.util.function.DoubleSupplier;

public class LocalizationManager {
    // private final Encoder leftOdo;
    // private final Encoder rightOdo;
    // private final TwoWheelTrackingLocalizer localizer;

    public LocalizationManager(Encoder leftOdo, Encoder rightOdo, DoubleSupplier headingSupplier) {
        // this.leftOdo = leftOdo;
        // this.rightOdo = rightOdo;
        // this.localizer = new TwoWheelTrackingLocalizer(leftOdo, rightOdo, headingSupplier);
    }

    public void reset() {
        // leftOdo.reset();
        // rightOdo.reset();
    }

    public Pose2d updateAndGetPose() {
        // localizer.update();
        // return localizer.getPoseEstimate();
        return new Pose2d();
    }
}
