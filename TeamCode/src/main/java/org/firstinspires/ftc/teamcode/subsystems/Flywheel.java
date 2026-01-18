package org.firstinspires.ftc.teamcode.subsystems;
 
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
 
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.templates.Feature;

import java.util.function.Supplier;
 
public class Flywheel extends Feature {
 
    private final static double Tmotor = 20.0, Tshaft = 39.0;
    private DcMotorEx motor;
    private VoltageSensor voltage;
    public static double P = 0.4, D = 0.0, kV = 0.000006;
    private Supplier<Double> targetVel = () -> 0.0;
    private PIDF velPID = new PIDF(P,D, () -> (targetVel.get() * kV));
    private double currentVel, pow;
 
    public Flywheel (HardwareMap map) { 
        motor = map.get(DcMotorEx.class, "shooter");
 
        voltage = map.get(VoltageSensor.class, "Control Hub");
 
        velPID.setTolerance(20);
 
        // hw.add(motor);
    }
 
    @Override
    public void update() {
        // for (Hardware h : hw) {
        //     h.update();
        // }
 
        currentVel = motor.getVelocity() * (60.0 / 28.0) * (Tmotor / Tshaft);
        // tps -> rpm
 
        pow = velPID.calculate(currentVel, targetVel.get());
        pow /= voltage.getVoltage();
 
        motor.setPower(-pow);
    }
 
    public Flywheel setVelocity(double target) {
        targetVel = () -> Math.max(target, 0);
        velPID.setSetPoint(targetVel.get());
        return this;
    }
 
    public boolean atVelocity() {
        velPID.calculate();
        return velPID.atSetPoint();
    }
 
    public double getCurrentVel() {
        return currentVel;
    }
 
    @Override
    public String toString() {
        return "Shooter{" +
                "velPID=" + velPID.getPositionalError() +
                ", currentVel=" + currentVel +
                ", targetVel=" + targetVel +
                ", setPower=" + velPID.calculate(currentVel) +
                ", voltage=" + voltage.getVoltage() +
                ", pow=" + pow +
                '}';
    }
}
