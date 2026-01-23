package org.firstinspires.ftc.teamcode.subsystems;
 
import com.qualcomm.robotcore.hardware.HardwareMap;
 
import org.firstinspires.ftc.teamcode.util.statemachine.State;
import org.firstinspires.ftc.teamcode.util.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.statemachine.Transition;
 
import java.util.ArrayList;

public class Shooter {
    private StateMachine state;
 
    private Flywheel flywheel;
    private Hood hood;
    private Turret turret;
    private Pose2d pose, goal;

    public final static double IDLE_VEL = 0, IDLE_HOOD = 0.5;
    public final static double TARGET_VEL = 5000;

    private double manualHoodPosition = IDLE_HOOD;
    private double manualTurretAngle = 0;

    private boolean autoAimEnabled = true;
    private boolean requestShot;

    // ---- Auto-aim tuning (NEAR triangle) ----
    // TUNE: Update the three distances (in your field units) for the NEAR triangle.
    private static final double[] NEAR_DISTANCES = {24, 36, 48};
    // TUNE: Update hood positions (pitch) for the NEAR triangle distances above.
    private static final double[] NEAR_PITCH = {0.62, 0.55, 0.48};
    // TUNE: Update yaw offsets (degrees) for the NEAR triangle distances above.
    private static final double[] NEAR_YAW = {0.0, 0.5, 1.0};

    // ---- Auto-aim tuning (FAR triangle) ----
    // TUNE: Update the three distances (in your field units) for the FAR triangle.
    private static final double[] FAR_DISTANCES = {60, 72, 84};
    // TUNE: Update hood positions (pitch) for the FAR triangle distances above.
    private static final double[] FAR_PITCH = {0.42, 0.38, 0.34};
    // TUNE: Update yaw offsets (degrees) for the FAR triangle distances above.
    private static final double[] FAR_YAW = {1.5, 2.0, 2.5};

    public Shooter (HardwareMap map) {
        flywheel = new Flywheel(map);
        hood = new Hood(map);
        turret = new Turret(map);
 
        requestShot = false;
        manualTurretAngle = turret.getAngle();
        pose = new Pose2d(0, 0, Math.toRadians(0));
        goal = new Pose2d(0, 0, Math.toRadians(0));
  
        State[] states = createStates();
        state = new StateMachine(states);
    }

    private State[] createStates() {
        State[] states = new State[2];
 
        states[0] = new State("IDLE")
                .setDuring(() -> {
                    flywheel.setVelocity(IDLE_VEL);
                    hood.setPosition(manualHoodPosition);
                    turret.setAngle(manualTurretAngle);
                })
                .addTransition(new Transition(() -> requestShot, "ON"));
 
        states[1] = new State("ON")
                .setDuring(() -> {
                    flywheel.setVelocity(TARGET_VEL);
                    hood.setPosition(manualHoodPosition);
                    turret.setAngle(manualTurretAngle);
                })
                .setFallbackState("IDLE")
                .addTransition(new Transition(() -> !requestShot, "IDLE"));
 
        return states;
    }

    public void init() {
        state.start();
        flywheel.setVelocity(IDLE_VEL);
        hood.setPosition(IDLE_HOOD);
        turret.setAngle(turret.getAngle());
        manualHoodPosition = IDLE_HOOD;
        manualTurretAngle = turret.getAngle();
    }
 
    public void update() {
        flywheel.update();
        if (autoAimEnabled) {
            updateAutoAim();
        }
        state.run();
    }

    public void updatePose(Pose2d pos) {
        pose = pos;
    }

    public void updateGoal(Pose2d goale) {
        goal = goale;
    }

    public void adjustHoodPosition(double delta) {
        manualHoodPosition = clamp(manualHoodPosition + delta, 0, 1);
    }

    public void adjustTurretAngle(double delta) {
        manualTurretAngle = clamp(manualTurretAngle + delta, 0, turret.SERVO_TO_ANGLE);
    }

    public void setAutoAimEnabled(boolean enabled) {
        autoAimEnabled = enabled;
    }

    public void setFlywheelEnabled(boolean enabled) {
        requestShot = enabled;
    }

    public boolean isFlywheelEnabled() {
        return requestShot;
    }

    private double getHoodFromFlywheel(double velocity) {
        return 1 - (velocity / 10000);
    }

    private void updateAutoAim() {
        if (pose == null || goal == null) {
            return;
        }
        double dx = goal.x - pose.x;
        double dy = goal.y - pose.y;
        double distance = Math.hypot(dx, dy);

        double[] pitchProfile;
        double[] yawProfile;
        double[] distanceProfile;

        if (distance <= NEAR_DISTANCES[NEAR_DISTANCES.length - 1]) {
            distanceProfile = NEAR_DISTANCES;
            pitchProfile = NEAR_PITCH;
            yawProfile = NEAR_YAW;
        } else {
            distanceProfile = FAR_DISTANCES;
            pitchProfile = FAR_PITCH;
            yawProfile = FAR_YAW;
        }

        double pitch = interpolateThreePoint(distance, distanceProfile, pitchProfile);
        double yawOffset = interpolateThreePoint(distance, distanceProfile, yawProfile);

        double baseYawDegrees = Math.toDegrees(Math.atan2(dy, dx)) - Math.toDegrees(pose.heading);
        manualTurretAngle = clamp(baseYawDegrees + yawOffset, 0, turret.SERVO_TO_ANGLE);
        manualHoodPosition = clamp(pitch, 0, 1);
    }

    private double interpolateThreePoint(double x, double[] xs, double[] ys) {
        if (xs.length != 3 || ys.length != 3) {
            throw new IllegalArgumentException("Auto-aim profiles must contain three points.");
        }
        if (x <= xs[0]) {
            return ys[0];
        }
        if (x >= xs[2]) {
            return ys[2];
        }
        if (x <= xs[1]) {
            return lerp(xs[0], ys[0], xs[1], ys[1], x);
        }
        return lerp(xs[1], ys[1], xs[2], ys[2], x);
    }

    private double lerp(double x0, double y0, double x1, double y1, double x) {
        double t = (x - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    public boolean isReady() {
        if (!state.currentState().equals("ON")) return false;
 
        return flywheel.atVelocity() && hood.atPosition() && turret.inPosition();
    }

    public void requestShot() {
        requestShot = true;
    }
 
    public void requestIdle() {
        requestShot = false;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public String toString(){
        return "Shooter {" + 
            flywheel.toString() + 
            hood.toString() + 
            turret.toString() + "}";
    }
}
