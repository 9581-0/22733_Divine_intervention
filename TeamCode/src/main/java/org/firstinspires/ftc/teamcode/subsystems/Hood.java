package org.firstinspires.ftc.teamcode.subsystems;
 
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
 
import org.firstinspires.ftc.teamcode.util.templates.Feature;
import org.firstinspires.ftc.teamcode.util.Utils;
 
import java.util.ArrayList;
 
public class Hood {
 
    private Servo servo;
    private boolean needsUpdate = false;
    private double pos = 0;

    private static final double SERVOMIN = 0, SERVOMAX = 0.5;
    private static final double ANGLEMIN = 30, ANGLEMAX = 60;
    private static final double SERVO_TO_ANGLE = 10;
 
    public Hood (HardwareMap map) {
        servo = map.get(Servo.class, "hood");
    }

    public void setPosition(double target){
        pos = Utils.minMaxClip(target, SERVOMIN, SERVOMAX);
        // servo.setPosition(pos);
    }
 
    public void setAngle(double target) {
        setPosition(angleToServo(target));
    }

    public double angleToServo(double angle) {
        return Utils.minMaxClip((angle - ANGLEMIN) / SERVO_TO_ANGLE + SERVOMIN, SERVOMIN, SERVOMAX);
    }

    public double servoToAngle(double position) {
        return Utils.minMaxClip((position - SERVOMIN) * SERVO_TO_ANGLE + ANGLEMIN, ANGLEMIN, ANGLEMAX);
    }

    public double getAngle(){
        return servoToAngle(pos);
    }

    public boolean atPosition(){
        return Math.abs(servo.getPosition() - pos) < 0.03;
    }
 
    @Override
    public String toString() {
        return "Hood{" +
                ", targetpos=" + pos +
                ", servopos=" + servo.getPosition() +
                '}';
    }
}
