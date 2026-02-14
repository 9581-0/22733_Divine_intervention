package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Shooter Velocity Tuner", group = "Test")
public class ShootHoodTuner extends OpMode {

    // ===== Hardware name in the Robot Configuration =====
    private static final String SHOOTER_MOTOR_NAME = "shooter";

    // ===== Tuning defaults =====
    private double targetVelocityTicksPerSec = 0;     // start stopped
    private double step = 1;                         // amount to change each button press (ticks/sec)

    private DcMotorEx shooter;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // For edge-detect (so it increments once per press, not every loop)
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME);

        // Reverse because your motor is mounted on the left (so flywheel spins correct direction)
        shooter.setDirection(DcMotor.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Use encoder velocity control
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Shooter Velocity Tuner ready.");
        telemetry.addLine("DPAD_UP/DOWN: +/- step");
        telemetry.addLine("X/Y: step smaller/larger");
        telemetry.addLine("A: start (apply target) | B: stop (0)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Button edges ---
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        // --- Step size adjust ---
        if (x && !lastX) step = Math.max(1, step / 2.0);      // smaller step
        if (y && !lastY) step = Math.min(5000, step * 2.0);   // larger step

        // --- Target changes ---
        if (dpadUp && !lastDpadUp) targetVelocityTicksPerSec += step;
        if (dpadDown && !lastDpadDown) targetVelocityTicksPerSec = Math.max(0, targetVelocityTicksPerSec - step);

        // Quick start/stop
        if (a && !lastA) {
            shooter.setVelocity(targetVelocityTicksPerSec);
        }
        if (b && !lastB) {
            targetVelocityTicksPerSec = 0;
            shooter.setVelocity(0);
        }

        // Continuously re-apply velocity so it stays locked even if you keep changing target
        shooter.setVelocity(targetVelocityTicksPerSec);

        // --- Telemetry ---
        double measuredVel = shooter.getVelocity(); // ticks/sec
        double error = targetVelocityTicksPerSec - measuredVel;

        telemetry.addData("Target (ticks/sec)", "%.1f", targetVelocityTicksPerSec);
        telemetry.addData("Measured (ticks/sec)", "%.1f", measuredVel);
        telemetry.addData("Error", "%.1f", error);
        telemetry.addData("Step", "%.1f", step);
        telemetry.addData("Motor Dir", shooter.getDirection());
        telemetry.addData("Mode", shooter.getMode());
        telemetry.addData("Loop ms", "%.1f", loopTimer.milliseconds());
        telemetry.update();
        loopTimer.reset();

        // --- Save last states ---
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastA = a;
        lastB = b;
        lastX = x;
        lastY = y;
    }
}



