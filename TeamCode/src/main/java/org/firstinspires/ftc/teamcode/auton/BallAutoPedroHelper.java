package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.pathing.PedroPathing;
import org.firstinspires.ftc.teamcode.util.vision.LimelightBallTracker;
import org.firstinspires.ftc.teamcode.util.vision.LimelightBallTracker.BallColor;
import org.firstinspires.ftc.teamcode.util.vision.LimelightBallTracker.BallDetection;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class BallAutoPedroHelper {
    private static final double DRIVE_SPEED = 0.5;
    private static final double BALL_REACH_DISTANCE = 3.0;

    private BallAutoPedroHelper() {
    }

    public static void runAuto(LinearOpMode opMode, BallColor allianceColor) {
        Robot robot = new Robot(opMode.hardwareMap);
        SwerveDrive swerve = new SwerveDrive(opMode.telemetry, opMode.hardwareMap);
        GoBildaPinpointDriver odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        LimelightBallTracker limelight = new LimelightBallTracker(opMode.hardwareMap);

        opMode.waitForStart();
        if (opMode.isStopRequested()) {
            return;
        }

        robot.init();
        robot.enableSort();
        robot.updateTelemetry(opMode.telemetry);

        while (opMode.opModeIsActive()) {
            Pose2d pose = odo.getPosition();
            List<BallDetection> detections = limelight.getBallDetections();
            List<Pose2d> ballTargets = toFieldTargets(detections, pose);

            ballTargets.sort(Comparator.comparingDouble(pose::getDistanceFromPoint));
            Pose2d target = ballTargets.isEmpty() ? null : ballTargets.get(0);

            if (target != null) {
                PedroPathing.DriveCommand command = PedroPathing.driveToPose(pose, target, DRIVE_SPEED);
                swerve.driveWithConfig(command.strafe, command.forward, command.rot);

                if (pose.getDistanceFromPoint(target) < BALL_REACH_DISTANCE) {
                    robot.requestIntake();
                }
            } else {
                swerve.driveWithConfig(0, 0, 0);
            }

            autoSortFromVision(robot, detections, allianceColor);
            robot.update();

            opMode.telemetry.addData("Alliance", allianceColor);
            opMode.telemetry.addData("Ball targets", ballTargets.size());
            opMode.telemetry.addData("Pose", pose);
            opMode.telemetry.update();
        }
    }

    private static List<Pose2d> toFieldTargets(List<BallDetection> detections, Pose2d pose) {
        List<Pose2d> targets = new ArrayList<>();
        double heading = pose.heading;
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        for (BallDetection detection : detections) {
            double fieldX = pose.x + (detection.relativeX * cos - detection.relativeY * sin);
            double fieldY = pose.y + (detection.relativeX * sin + detection.relativeY * cos);
            targets.add(new Pose2d(fieldX, fieldY, pose.heading));
        }
        return targets;
    }

    private static void autoSortFromVision(Robot robot, List<BallDetection> detections, BallColor alliance) {
        boolean opponentSeen = false;
        for (BallDetection detection : detections) {
            if (detection.color != BallColor.UNKNOWN && detection.color != alliance) {
                opponentSeen = true;
                break;
            }
        }
        if (opponentSeen) {
            robot.enableSort();
        } else {
            robot.disableSort();
        }
    }
}
