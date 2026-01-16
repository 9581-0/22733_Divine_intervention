package org.firstinspires.ftc.teamcode.util.wrappers;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Sensorange{

    private AnalogInput encoder;
    private double lastCheckedPosition, fullRotations;
    private Double currentAbsoluteAngle;


    public Sensorange (String name, HardwareMap map) {
        encoder = map.get(AnalogInput.class, name);
        // sensorange
        lastCheckedPosition = AngleUnit.normalizeDegrees((encoder.getVoltage() - 0.043) / 3.1 * 360);
        currentAbsoluteAngle = lastCheckedPosition;
    }

    @Override
    public void calculateValue() {
        double encoderResult = (encoder.getVoltage() / 3.3) * 360;

        if (lastCheckedPosition >= 180 && encoderResult <= 180) {
            fullRotations++;
        } else if (lastCheckedPosition <= 180 && encoderResult >= 180) {
            fullRotations--;
        }

        if (currentAbsoluteAngle != null) {
            currentAbsoluteAngle = encoderResult + fullRotations * 360;
        }

        lastCheckedPosition = encoderResult;
    }

    @Override
    public Double getValue() {
        return lastCheckedPosition;
    }

    @NonNull
    @Override
    public String toString() {
        return "current absolute angle " + currentAbsoluteAngle + "\n" +
                "current relative position " + lastCheckedPosition + "\n" +
                "full rotations " + fullRotations  + "\n"
                ;
    }

    @Override
    public double getPosition() {
        return currentAbsoluteAngle;
    }
}
