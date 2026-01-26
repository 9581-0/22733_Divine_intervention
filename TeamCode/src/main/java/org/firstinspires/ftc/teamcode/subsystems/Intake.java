package org.firstinspires.ftc.teamcode.subsystems;
 
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
 
import org.firstinspires.ftc.teamcode.utility.myDcMotorEx;
import org.firstinspires.ftc.teamcode.util.templates.Feature;
 
import java.util.ArrayList;
 
public class Intake extends Feature {
 
    final private myDcMotorEx motor;
    private boolean needsUpdate = false;
    private double pow = 0;
 
    public Intake (HardwareMap map) {
        motor = new myDcMotorEx(map.get(DcMotorEx.class, "intake"));
    }
 
    @Override
    public void update() {
        if (needsUpdate) {
            motor.setPower(pow);
            needsUpdate = false;
        }
    }
 
    public void setPower(double target) {
        pow = target;
        needsUpdate = true;
    }
 
    @Override
    public String toString() {
        return "Intake{" +
                ", pow=" + pow +
                ", actual pow = " + motor.getPower() +
                '}';
    }
}
