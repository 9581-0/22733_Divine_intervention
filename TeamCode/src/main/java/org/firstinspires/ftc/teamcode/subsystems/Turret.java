package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;

public class Turret {
    public static final double GEAR_RATIO = (double) 80 /30* (double) 37/112, SERVO_TO_ANGLE = 216/255.0*355;
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

             double angleToGoal = Math.atan2(GOAL_X - currentX, GOAL_Y - currentY) + 3*Math.PI/2;

             setPos((angleToGoal - currentH) / SERVO_TO_ANGLE + 0.5);
         }
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

    public double getAngle(){
        return LServo.getPosition() * SERVO_TO_ANGLE;
    }

    public void setAngle(double angle){
        setPos(angle / SERVO_TO_ANGLE);
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
