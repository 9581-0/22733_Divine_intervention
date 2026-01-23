package org.firstinspires.ftc.teamcode.util.pathing;

import org.firstinspires.ftc.teamcode.util.Pose2d;

public class PedroPathing {
    private static final double POSITION_TOLERANCE = 1.0;

    public static DriveCommand driveToPose(Pose2d current, Pose2d target, double speed) {
        double dx = target.x - current.x;
        double dy = target.y - current.y;
        double distance = Math.hypot(dx, dy);

        if (distance < POSITION_TOLERANCE) {
            return new DriveCommand(0, 0, 0, true);
        }

        double forward = clamp(dx / distance, -1, 1) * speed;
        double strafe = clamp(dy / distance, -1, 1) * speed;
        return new DriveCommand(strafe, forward, 0, false);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static class DriveCommand {
        public final double strafe;
        public final double forward;
        public final double rot;
        public final boolean reachedTarget;

        public DriveCommand(double strafe, double forward, double rot, boolean reachedTarget) {
            this.strafe = strafe;
            this.forward = forward;
            this.rot = rot;
            this.reachedTarget = reachedTarget;
        }
    }
}
