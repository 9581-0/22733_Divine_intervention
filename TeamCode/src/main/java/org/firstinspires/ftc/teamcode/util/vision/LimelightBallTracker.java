package org.firstinspires.ftc.teamcode.util.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class LimelightBallTracker {
    private final Object limelight;

    public LimelightBallTracker(HardwareMap hardwareMap) {
        Object device = null;
        try {
            Class<?> limelightClass = Class.forName("com.qualcomm.hardware.limelight.Limelight3A");
            device = hardwareMap.get((Class<Object>) limelightClass, "limelight");
        } catch (Exception ignored) {
            device = null;
        }
        limelight = device;
    }

    public List<BallDetection> getBallDetections() {
        if (limelight == null) {
            return Collections.emptyList();
        }

        try {
            Method latestResultMethod = limelight.getClass().getMethod("getLatestResult");
            Object latestResult = latestResultMethod.invoke(limelight);
            if (latestResult == null) {
                return Collections.emptyList();
            }

            Method detectorResultsMethod = latestResult.getClass().getMethod("getDetectorResults");
            Object detectorResults = detectorResultsMethod.invoke(latestResult);
            if (!(detectorResults instanceof List)) {
                return Collections.emptyList();
            }

            List<BallDetection> detections = new ArrayList<>();
            for (Object result : (List<?>) detectorResults) {
                Double x = getDouble(result, "getRobotRelativeX", "getTargetX", "getX");
                Double y = getDouble(result, "getRobotRelativeY", "getTargetY", "getY");
                String label = getString(result, "getClassName", "getName", "getLabel");
                if (x != null && y != null) {
                    detections.add(new BallDetection(x, y, BallColor.fromLabel(label)));
                }
            }
            return detections;
        } catch (Exception ignored) {
            return Collections.emptyList();
        }
    }

    private Double getDouble(Object target, String... methodNames) {
        for (String methodName : methodNames) {
            try {
                Method method = target.getClass().getMethod(methodName);
                Object value = method.invoke(target);
                if (value instanceof Number) {
                    return ((Number) value).doubleValue();
                }
            } catch (Exception ignored) {
                // ignore and try next method name
            }
        }
        return null;
    }

    private String getString(Object target, String... methodNames) {
        for (String methodName : methodNames) {
            try {
                Method method = target.getClass().getMethod(methodName);
                Object value = method.invoke(target);
                if (value != null) {
                    return value.toString();
                }
            } catch (Exception ignored) {
                // ignore and try next method name
            }
        }
        return null;
    }

    public enum BallColor {
        RED,
        BLUE,
        UNKNOWN;

        public static BallColor fromLabel(String label) {
            if (label == null) {
                return UNKNOWN;
            }
            String normalized = label.toLowerCase();
            if (normalized.contains("red")) {
                return RED;
            }
            if (normalized.contains("blue")) {
                return BLUE;
            }
            return UNKNOWN;
        }
    }

    public static class BallDetection {
        public final double relativeX;
        public final double relativeY;
        public final BallColor color;

        public BallDetection(double relativeX, double relativeY, BallColor color) {
            this.relativeX = relativeX;
            this.relativeY = relativeY;
            this.color = color;
        }
    }
}
