package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Lightweight wrapper around a motor-based encoder for odometry pods.
 */
public class Encoder {
    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        final int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }
    }

    private final DcMotorEx motor;
    private int directionMultiplier = 1;

    public Encoder(@NonNull DcMotorEx motor) {
        this.motor = motor;
    }

    public Encoder(@NonNull HardwareMap hardwareMap, @NonNull String name) {
        this((DcMotorEx) hardwareMap.get(DcMotor.class, name));
    }

    public void setDirection(Direction direction) {
        directionMultiplier = direction.multiplier;
    }

    public double getCurrentPosition() {
        return motor.getCurrentPosition() * directionMultiplier;
    }

    public double getCorrectedVelocity() {
        return motor.getVelocity() * directionMultiplier;
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    public void reset() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setHardwareDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }
}
