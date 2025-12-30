package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class Vision_Camera {


    private final CRServo ServoL;
    private final CRServo ServoR;


    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;

    private boolean trackingEnabled = false;

    private static final double kP = 0.015;
    private static final double MAX_POWER = 0.4;
    private static final double DEADBAND_DEGREES = 1.0;

    public Vision_Camera(HardwareMap hardwareMap) {


        ServoL = hardwareMap.get(CRServo.class, "ServoL");
        ServoL.setDirection(DcMotorSimple.Direction.FORWARD);

        ServoR = hardwareMap.get(CRServo.class, "ServoR");
        ServoR.setDirection(DcMotorSimple.Direction.FORWARD);

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Vision portal
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );
    }


    public void enableTracking() {
        trackingEnabled = true;
    }

    public void disableTracking() {
        trackingEnabled = false;
        ServoL.setPower(0);
        ServoR.setPower(0);
    }

    public boolean isTracking() {
        return trackingEnabled;
    }

    public void update() {
        if (!trackingEnabled) {
            ServoL.setPower(0);
            ServoR.setPower(0);
            return;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.isEmpty()) {
            ServoL.setPower(0);
            ServoR.setPower(0);
            return;
        }

        AprilTagDetection tag = detections.get(0);

        double yawError = tag.ftcPose.yaw; // degrees

        double power = 0.0;
        if (Math.abs(yawError) > DEADBAND_DEGREES) {
            power = yawError * kP;
        }

        // Clamp power
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        ServoL.setPower(power);
        ServoR.setPower(power);
    }
}