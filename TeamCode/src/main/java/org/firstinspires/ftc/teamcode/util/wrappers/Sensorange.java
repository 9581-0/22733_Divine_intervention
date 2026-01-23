package org.firstinspires.ftc.teamcode.util.wrappers;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Sensorange{

    private AnalogInput encoder;
    private double lastCheckedPosition = 0, fullRotations = 0;
    private double currentAbsoluteAngle = 0;


    public Sensorange (String name, HardwareMap map) {
        encoder = map.get(AnalogInput.class, name);
        // sensorange
        lastCheckedPosition = AngleUnit.normalizeDegrees((encoder.getVoltage() - 0.043) / 3.1 * 360);
        currentAbsoluteAngle = lastCheckedPosition;
    }

    public void calculateValue() {
        double encoderResult = AngleUnit.normalizeDegrees((encoder.getVoltage() - 0.043) / 3.1 * 360);

        if (lastCheckedPosition - encoderResult > 200) {
            fullRotations++;
        } else if (lastCheckedPosition - encoderResult < -200) {
            fullRotations--;
        }

        currentAbsoluteAngle = encoderResult + fullRotations * 360;

        lastCheckedPosition = encoderResult;
    }

    public double getValue() {
        return lastCheckedPosition;
    }

    @NonNull
    public String toString() {
        return "current absolute angle " + currentAbsoluteAngle + "\n" +
                "current relative position " + lastCheckedPosition + "\n" +
                "full rotations " + fullRotations  + "\n"
                ;
    }

    public double getPosition() {
        return currentAbsoluteAngle;
    }
}
