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

    private boolean requestShot;
 
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

    public void setFlywheelEnabled(boolean enabled) {
        requestShot = enabled;
    }

    public boolean isFlywheelEnabled() {
        return requestShot;
    }

    private double getHoodFromFlywheel(double velocity) {
        return 1 - (velocity / 10000);
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
