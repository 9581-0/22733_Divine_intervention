package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
//@Disabled
public class FlywheelPIDFTuner extends OpMode {

    public DcMotorEx flywheelMotor;

    public double highVelocity = 1600;
    public double lowVelocity  = 1400;

    private double curTargetVelocity = highVelocity;

    // These are the values you tune with the gamepad
    private double F = 0;
    private double P = 0;

    // Keep your existing step sizes (though F usually ends up VERY small in practice)
    private final double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    private int stepIndex = 1;

    // Only re-send PIDF when it changes
    private double lastP = Double.NaN;
    private double lastF = Double.NaN;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Keep the physical direction you need
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        applyPidfIfChanged();

        telemetry.addLine("Init complete");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Toggle target velocity between high and low
        if (gamepad1.yWasPressed()) {
            curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
        }

        // Cycle step size
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Tune F
        if (gamepad1.dpadLeftWasPressed())  F -= stepSizes[stepIndex];
        if (gamepad1.dpadRightWasPressed()) F += stepSizes[stepIndex];

        // Tune P
        if (gamepad1.dpadDownWasPressed()) P -= stepSizes[stepIndex];
        if (gamepad1.dpadUpWasPressed())   P += stepSizes[stepIndex];

        applyPidfIfChanged();

        // Command POSITIVE velocity; motor direction handles physical spin
        flywheelMotor.setVelocity(curTargetVelocity);

        double rawVelocity = flywheelMotor.getVelocity();
        double speed = Math.abs(rawVelocity);
        double error = curTargetVelocity - speed;

        telemetry.addData("Target Velocity (tps)", curTargetVelocity);
        telemetry.addData("Current Velocity RAW (tps)", "%.2f", rawVelocity);
        telemetry.addData("Current Speed |vel| (tps)", "%.2f", speed);
        telemetry.addData("Error (target - |vel|)", "%.2f", error);
        telemetry.addLine("------------------------------");
        telemetry.addData("Pos", flywheelMotor.getCurrentPosition());
        telemetry.addLine("------------------------------");
        telemetry.addData("Tuning P", "%.6f (D-Pad U/D)", P);
        telemetry.addData("Tuning F (UI)", "%.6f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B button)", stepSizes[stepIndex]);

        telemetry.update();
    }

    private void applyPidfIfChanged() {
        if (P != lastP || F != lastF) {
            /*
             * CRITICAL FIX:
             * Your system’s kF sign is inverted. If increasing F makes it spin backwards,
             * you need to APPLY kF with the opposite sign.
             */
            double appliedF = -F;

            PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, appliedF);
            flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

            lastP = P;
            lastF = F;
        }
    }
}


