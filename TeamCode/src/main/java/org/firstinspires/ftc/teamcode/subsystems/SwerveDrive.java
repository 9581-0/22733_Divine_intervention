package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.maths.mathsOperations;
import org.firstinspires.ftc.teamcode.maths.swerveKinematics;
import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.utility.myDcMotorEx;
import org.firstinspires.ftc.teamcode.teleop.SwerveTeleOpConfig;

public class SwerveDrive {

    private GoBildaPinpointDriver odo;
    private final myDcMotorEx mod1m1, mod1m2, mod2m1, mod2m2, mod3m1, mod3m2;
    private final AnalogInput mod1E, mod2E, mod3E;
    private Telemetry telemetry;

    // PIDs
    private final PIDcontroller mod1PID = new PIDcontroller(0,0,0,0,0);
    private final PIDcontroller mod2PID = new PIDcontroller(0,0,0,0,0);
    private final PIDcontroller mod3PID = new PIDcontroller(0,0,0,0,0);

    private final swerveKinematics kinematics = new swerveKinematics();

    private double imuOffset = 180;

    private static final double HEADING_LOCK_KP = 0.002;
    private static final double HEADING_LOCK_KI = 0.00002;
    private static final double HEADING_LOCK_KD = 0.0001;
    private static final double HEADING_LOCK_KF = 0.00005;
    private static final double HEADING_LOCK_KL = 0.1;
    private static final double HEADING_LOCK_DEADBAND = 1e-3;

    private final PIDcontroller headingLockPID = new PIDcontroller(
            HEADING_LOCK_KP,
            HEADING_LOCK_KD,
            HEADING_LOCK_KI,
            HEADING_LOCK_KF,
            HEADING_LOCK_KL
    );
    private double headingLockTarget = 0;
    private boolean headingLockActive = false;

    // Safety Variables
    private boolean initialized = false;
    private double lastGoodHeading = 180; // Failsafe for IMU singularity

    public SwerveDrive(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        // --- Hardware Mapping ---
        mod1m1 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod1m1"));
        mod1m2 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod1m2"));
        mod2m1 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod2m1"));
        mod2m2 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod2m2"));
        mod3m1 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod3m1"));
        mod3m2 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod3m2"));

        mod1E = hardwareMap.get(AnalogInput.class, "mod1E");
        mod2E = hardwareMap.get(AnalogInput.class, "mod2E");
        mod3E = hardwareMap.get(AnalogInput.class, "mod3E");

        // --- Motor Defaults ---
        myDcMotorEx[] allMotors = {mod1m1, mod1m2, mod2m1, mod2m2, mod3m1, mod3m2};
        for (myDcMotorEx motor : allMotors) {
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setPowerThresholds(0.05, 0);
        }

        // --- Motor Reversals ---
        // Only bottom motors are reversed (Coaxial standard)
        mod1m2.setDirection(DcMotorSimple.Direction.FORWARD);
        mod2m2.setDirection(DcMotorSimple.Direction.FORWARD);
        mod3m2.setDirection(DcMotorSimple.Direction.FORWARD);


    }

    public void driveWithConfig(double strafe, double forward, double rot){
        drive(strafe, forward, rot, SwerveTeleOpConfig.module1Adjust,
                SwerveTeleOpConfig.module2Adjust,
                SwerveTeleOpConfig.module3Adjust,
                SwerveTeleOpConfig.Kp,
                SwerveTeleOpConfig.Kd,
                SwerveTeleOpConfig.Ki,
                SwerveTeleOpConfig.Kf,
                SwerveTeleOpConfig.Kl,
                SwerveTeleOpConfig.FIELD_CENTRIC,
                SwerveTeleOpConfig.IMU_POLARITY,
                SwerveTeleOpConfig.ROBOT_RADIUS);
    }

    public void drive(double strafe, double forward, double rot,
                      double m1Offset, double m2Offset, double m3Offset,
                      double Kp, double Kd, double Ki, double Kf, double Kl,
                      boolean fieldCentric, double imuPolarity, double robotRadius) {

        // 0. Safety: Sanitize Inputs (Prevent NaN from crashing PIDs)
        if (Double.isNaN(forward) || Double.isNaN(strafe) || Double.isNaN(rot)) {
            forward = 0; strafe = 0; rot = 0;
        }

        // 1. Update PID Coeffs
        mod1PID.setPIDgains(Kp, Kd, Ki, Kf, Kl);
        mod2PID.setPIDgains(Kp, Kd, Ki, Kf, Kl);
        mod3PID.setPIDgains(Kp, Kd, Ki, Kf, Kl);

        // 2. Read Sensors (Volts -> Degrees)
        double mod1P = readEncoder(mod1E, m1Offset);
        double mod2P = readEncoder(mod2E, m2Offset);
        double mod3P = readEncoder(mod3E, m3Offset);

        // 3. Get Heading (With Singularity Failsafe)
        double heading = 0;
        boolean needsHeading = fieldCentric || (initialized && Math.abs(rot) < HEADING_LOCK_DEADBAND);
        if (needsHeading) {
            if (odo == null) {
                heading = lastGoodHeading;
                telemetry.addData("WARNING", "IMU UNAVAILABLE - USING FALLBACK");
            } else {
                double rawHeading = getHeading(imuPolarity);

                // Check if IMU returned NaN (Singularity)
                if (Double.isNaN(rawHeading)) {
                    heading = lastGoodHeading; // Use last known safe angle
                    telemetry.addData("WARNING", "IMU NAN DETECTED - USING FALLBACK");
                } else {
                    heading = rawHeading;
                    lastGoodHeading = heading; // Update history
                }
            }
        }

        // 4. Heading Lock PIDF (Apply when rotation stick is released)
        if (initialized && Math.abs(rot) < HEADING_LOCK_DEADBAND) {
            if (!headingLockActive) {
                headingLockTarget = heading;
                headingLockPID.reset();
                headingLockActive = true;
            }
            double headingError = AngleUnit.normalizeDegrees(headingLockTarget - heading);
            rot = headingLockPID.pidOut(headingError);
        } else {
            if (headingLockActive) {
                headingLockPID.reset();
            }
            headingLockActive = false;
            headingLockTarget = heading;
        }

        // 5. Calculate Kinematics (Vectors)
        double[] output = kinematics.calculate(forward, -strafe, -rot, heading, fieldCentric, robotRadius);

        double mod1power = output[0];
        double mod2power = output[1];
        double mod3power = output[2];

        // References
        double mod1reference = output[3];
        double mod2reference = output[4];
        double mod3reference = output[5];

        // 6. Locking Logic
        if (forward != 0 || strafe != 0 || rot != 0 || !initialized) {
            initialized = true;
        }

        // 7. Efficient Turn & PID
        // Angle Wrap
        mod1P = mathsOperations.angleWrap(mod1P);
        mod2P = mathsOperations.angleWrap(mod2P);
        mod3P = mathsOperations.angleWrap(mod3P);

        mod1reference = mathsOperations.angleWrap(mod1reference);
        mod2reference = mathsOperations.angleWrap(mod2reference);
        mod3reference = mathsOperations.angleWrap(mod3reference);

        // Efficient Turn
        double[] m1Eff = mathsOperations.efficientTurn(mod1reference, mod1P, mod1power);
        double[] m2Eff = mathsOperations.efficientTurn(mod2reference, mod2P, mod2power);
        double[] m3Eff = mathsOperations.efficientTurn(mod3reference, mod3P, mod3power);

        // PID Calculation
        double m1PID = mod1PID.pidOut(AngleUnit.normalizeDegrees(m1Eff[0] - mod1P));
        double m2PID = mod2PID.pidOut(AngleUnit.normalizeDegrees(m2Eff[0] - mod2P));
        double m3PID = mod3PID.pidOut(AngleUnit.normalizeDegrees(m3Eff[0] - mod3P));

        // 8. Differential Mixing (PID + Drive Power)
        double[] m1Out = mathsOperations.diffyConvert(-m1PID, m1Eff[1]);
        double[] m2Out = mathsOperations.diffyConvert(-m2PID, m2Eff[1]);
        double[] m3Out = mathsOperations.diffyConvert(-m3PID, m3Eff[1]);

        // 9. Output
        mod1m1.setPower(m1Out[0]); mod1m2.setPower(m1Out[1]);
        mod2m1.setPower(m2Out[0]); mod2m2.setPower(m2Out[1]);
        mod3m1.setPower(m3Out[0]); mod3m2.setPower(m3Out[1]);

        // 10. Telemetry
        telemetry.addData("Heading", heading);
        telemetry.addData("M1 Angle/Ref", "%.1f / %.1f", mod1P, m1Eff[0]);
        telemetry.addData("M2 Angle/Ref", "%.1f / %.1f", mod2P, m2Eff[0]);
        telemetry.addData("M3 Angle/Ref", "%.1f / %.1f", mod3P, m3Eff[0]);
    }

    private double readEncoder(AnalogInput enc, double offset) {
        // Parentheses here are critical! Do not remove.
        double raw = (enc.getVoltage() - 0.043) / 3.1 * 360.0;
        return AngleUnit.normalizeDegrees(raw - offset);
    }

    private double getHeading(double polarity) {
        return AngleUnit.normalizeDegrees(odo.getHeading(AngleUnit.DEGREES) * polarity - imuOffset);
    }

    public void resetIMU() {
        imuOffset = odo.getHeading(AngleUnit.DEGREES);
    }


    public void tuneModules(double targetAngle, double m1Offset, double m2Offset, double m3Offset,
                            double Kp, double Kd, double Ki, double Kf, double Kl) {
        // Update PIDs
        mod1PID.setPIDgains(Kp, Kd, Ki, Kf, Kl);
        mod2PID.setPIDgains(Kp, Kd, Ki, Kf, Kl);
        mod3PID.setPIDgains(Kp, Kd, Ki, Kf, Kl);

        // Read Current Angles
        double mod1P = readEncoder(mod1E, m1Offset);
        double mod2P = readEncoder(mod2E, m2Offset);
        double mod3P = readEncoder(mod3E, m3Offset);

        // Wrap/Optimize
        mod1P = mathsOperations.angleWrap(mod1P);
        mod2P = mathsOperations.angleWrap(mod2P);
        mod3P = mathsOperations.angleWrap(mod3P);

        double[] m1Eff = mathsOperations.efficientTurn(targetAngle, mod1P, 0);
        double[] m2Eff = mathsOperations.efficientTurn(targetAngle, mod2P, 0);
        double[] m3Eff = mathsOperations.efficientTurn(targetAngle, mod3P, 0);

        double m1PID = mod1PID.pidOut(AngleUnit.normalizeDegrees(m1Eff[0] - mod1P));
        double m2PID = mod2PID.pidOut(AngleUnit.normalizeDegrees(m2Eff[0] - mod2P));
        double m3PID = mod3PID.pidOut(AngleUnit.normalizeDegrees(m3Eff[0] - mod3P));

        // Apply ONLY rotation power (no drive)
        double[] m1Out = mathsOperations.diffyConvert(m1PID, 0);
        double[] m2Out = mathsOperations.diffyConvert(-m2PID, 0);
        double[] m3Out = mathsOperations.diffyConvert(-m3PID, 0);

        mod1m1.setPower(m1Out[0]); mod1m2.setPower(m1Out[1]);
        mod2m1.setPower(m2Out[0]); mod2m2.setPower(m2Out[1]);
        mod3m1.setPower(m3Out[0]); mod3m2.setPower(m3Out[1]);

        telemetry.addData("Target", targetAngle);
        telemetry.addData("M1 Pos", mod1P);
        telemetry.addData("M2 Pos", mod2P);
        telemetry.addData("M3 Pos", mod3P);
    }

    public void updateTelemetry(Telemetry telem){
        telemetry = telem;
    }

    public void updateOdo(GoBildaPinpointDriver odom){
        odo = odom;
    }
}
