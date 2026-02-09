package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;

public class Turret {
    public static final double GEAR_RATIO = (double) 80 /30* (double) 37/112;
    public static double GOAL_X = 0.0, GOAL_Y = 0.0;

    private final Servo LServo, RServo;

    public double target;
    public static boolean tracking = false;

    public Turret (HardwareMap map) {
        LServo = map.get(Servo.class, "TurretServoL");
        RServo = map.get(Servo.class, "TurretServoR");
    }


     public void update(Pose2d currentPosition){
         if (tracking) {
             double currentX = currentPosition.getX();
             double currentY = currentPosition.getY();
             double currentH = currentPosition.getHeading();

             double angleToGoal = Math.atan2(GOAL_X - currentX, GOAL_Y - currentY) + Math.PI/2;

             setPos(clamp((angleToGoal - currentH) / ( GEAR_RATIO) + 0.5, 1));
         }
     }

     public double clamp(double t, double range) {
        if(t > range) {
            return range;
        }
        else if(t < -range) {
            return -range;
        }
        return t;
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
