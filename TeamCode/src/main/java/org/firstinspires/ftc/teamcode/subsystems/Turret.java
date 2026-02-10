package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;

public class Turret {
    public static final double GEAR_RATIO = (double) 80 /20* (double) 37/112;
    public static double GOAL_X = 0.0, GOAL_Y = 0.0;

    private final Servo LServo, RServo;

    public double target;
    public static boolean tracking = false;

    public Turret (HardwareMap map) {
        LServo = map.get(Servo.class, "TurretServoL");
        RServo = map.get(Servo.class, "TurretServoR");
    }


    public void update(Pose2d currentPosition) {
        if (!tracking) return;

        double dx = GOAL_X - currentPosition.getX();
        double dy = GOAL_Y - currentPosition.getY();

        double angleToGoal = Math.atan2(dy, dx);
        double error = angleWrap(angleToGoal - currentPosition.getHeading());

        double SERVO_RANGE_RAD = Math.toRadians(270);
        double servoDelta = (error  / GEAR_RATIO);

        double pos = clamp(0.5 + servoDelta);
        setPos(pos);
    }

    public double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }


    private double angleWrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    public void sotm() {

     }

     public void setGoalPositions(double x, double y){
         GOAL_X = x;
         GOAL_Y = y;
     }

    private void setPos(double pos){
        LServo.setPosition(pos);
        RServo.setPosition(pos);
        target = pos;
    }

    public double getPosition(){
        return LServo.getPosition();
    }


    public void setPosition(double pos){
        setPos(pos);
    }

    public boolean inPosition(){
        return Math.abs(LServo.getPosition() - target) < 0.02;
    }

    public void setHalf() {
        tracking = false;
        setPos(0.5);
    }
}
