package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;

public class Turret {
    public static final double GEAR_RATIO = 80/30*37/112, SERVO_TO_ANGLE = 300;
    public static double GOAL_X = 0.0, GOAL_Y = 0.0;

    private final Servo LServo, RServo;

    public double target;
    private boolean tracking = false;

    public Turret (HardwareMap map) {
        LServo = map.get(Servo.class, "TurretServoL");
        RServo = map.get(Servo.class, "TurretServoR");
    }

     public void update(Pose2d currentPosition){
         if (tracking) {
             double currentX = currentPosition.getX();
             double currentY = currentPosition.getY();
             double currentH = currentPosition.getHeading(); // assumed 0 is facing goal side

             double angleToGoal = Math.atan2(GOAL_Y - currentY, GOAL_X - currentX);

             setPos((angleToGoal - currentH) / SERVO_TO_ANGLE + 0.5);
         }
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

    public void startTracking(){tracking = true;}
    public void stopTracking(){tracking = false;}
}
