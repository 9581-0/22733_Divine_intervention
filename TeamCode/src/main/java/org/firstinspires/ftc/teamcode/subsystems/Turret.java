package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Pose2d;

public class Turret {
    public static final double GEAR_RATIO = (90 / 20.0) * (37.0 / 112.0);

    public static final double SERVO_RANGE_RAD = Math.toRadians(355);

    public static double GOAL_X = 0.0, GOAL_Y = 0.0;

    private final Servo LServo, RServo;
    public double target = 0.5;
    public static boolean tracking = true;
    public static String snitch = "";

    public Turret (HardwareMap map) {
        LServo = map.get(Servo.class, "TurretServoL");
        RServo = map.get(Servo.class, "TurretServoR");
        setPos(0.5); // Initialize to center
    }

    public void update(Pose2d currentPosition) {
        if (!tracking) return;

        double dx = GOAL_X - currentPosition.getX();
        double dy = GOAL_Y - currentPosition.getY();

        // Calculate angle from robot to target
        double angleToGoal = Math.atan2(dy, dx);

        // Error is the difference between goal and current robot heading
        // We subtract Math.PI if the turret "zero" is facing the back of the robot
        double error = angleWrap(angleToGoal + currentPosition.getHeading());

        // Map the radian error to a 0.0 - 1.0 servo value using gear ratio
        double servoDelta = (error / GEAR_RATIO) / SERVO_RANGE_RAD;

        // Apply delta to the center position (0.5)
        double pos = clamp(0.5 + servoDelta);
        snitch = "atg " + angleToGoal + " error " + error + " servodelta " + servoDelta;
        setPos(pos);
    }

    public void setPos(double pos){
        target = pos;
        LServo.setPosition(pos);
        RServo.setPosition(pos);
    }

    public void setGoalPositions(double x, double y){
        GOAL_X = x;
        GOAL_Y = y;
    }

    public double getPosition(){
        return LServo.getPosition();
    }

    public void setPosition(double pos){
        tracking = false; // Disable auto-tracking when manual position is set
        setPos(pos);
    }

    public boolean inPosition(){
        return Math.abs(LServo.getPosition() - target) < 0.02;
    }

    public void setHalf() {
        tracking = false;
        setPos(0.5);
    }

    private double angleWrap(double a) {
        while (a > Math.PI * GEAR_RATIO) a -= 2 * Math.PI;
        while (a < -Math.PI * GEAR_RATIO) a += 2 * Math.PI;
        return a;
    }

    private double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    @Override
    public String toString(){
        return "Turret { " + target + "} snitch {" + snitch + "}";
    }
}
