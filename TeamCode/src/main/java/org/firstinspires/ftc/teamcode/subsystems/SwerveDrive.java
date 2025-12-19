package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.maths.mathsOperations;
import org.firstinspires.ftc.teamcode.maths.swerveKinematics;
import org.firstinspires.ftc.teamcode.utility.myDcMotorEx;

public class SwerveDrive {

    private final IMU imu;
    private final myDcMotorEx mod1m1, mod1m2, mod2m1, mod2m2, mod3m1, mod3m2;
    private final AnalogInput mod1E, mod2E, mod3E;
    private final Telemetry telemetry;

    // PIDs
    private final PIDcontroller mod1PID = new PIDcontroller(0,0,0,0,0);
    private final PIDcontroller mod2PID = new PIDcontroller(0,0,0,0,0);
    private final PIDcontroller mod3PID = new PIDcontroller(0,0,0,0,0);
    
    private final swerveKinematics kinematics = new swerveKinematics();

    // References
    private double mod1reference = 0, mod2reference = 0, mod3reference = 0;
    private double imuOffset = 0;
    private boolean initialized = false;

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
            motor.setPowerThresholds(0.05, 0); // Preserve your old deadzone logic
        }
        // --- Motor Reversals (FIXED) ---
        // Only bottom motors are reversed (Coaxial standard for Diffy Swerve usually)
        mod1m2.setDirection(DcMotorSimple.Direction.REVERSE);
        mod2m2.setDirection(DcMotorSimple.Direction.FORWARD);
        mod3m2.setDirection(DcMotorSimple.Direction.FORWARD);
        

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(params);
    }

    public void drive(double strafe, double forward, double rot, 
                      double m1Offset, double m2Offset, double m3Offset,
                      double Kp, double Kd, double Ki, double Kf, double Kl,
                      boolean fieldCentric, double imuPolarity, double robotRadius) {

        // 1. Update PID Coeffs
        mod1PID.setPIDgains(Kp, Kd, Ki, Kf, Kl);
        mod2PID.setPIDgains(Kp, Kd, Ki, Kf, Kl);
        mod3PID.setPIDgains(Kp, Kd, Ki, Kf, Kl);

        // 2. Read Sensors (Volts -> Degrees)
        // Formula: (V - 0.043) / 3.1 * 360. 
        // We add the offset HERE so the rest of the code works with "Real Degrees"
        double mod1P = readEncoder(mod1E, m1Offset);
        double mod2P = readEncoder(mod2E, m2Offset);
        double mod3P = readEncoder(mod3E, m3Offset);

        // 3. Get Heading
        double heading = 0;
        if (fieldCentric) {
            heading = getHeading(imuPolarity);
        }

        // 4. Calculate Kinematics (Vectors)
        // We pass the config variables into the kinematics calculator
        double[] output = kinematics.calculate(forward, -strafe, -rot, heading, fieldCentric, robotRadius);
        
        double mod1power = output[0];
        double mod3power = output[1]; // Kinematics returns mod1, mod3, mod2 order in your old code?
        double mod2power = output[2]; // Checked: Old code returned {v1, v2, v3, theta1, theta2, theta3}

        // 5. Locking Logic (Don't move wheels if joystick is dead)
        if (forward != 0 || strafe != 0 || rot != 0 || !initialized) {
            mod1reference = output[3];
            mod3reference = output[5]; // Order match: Kinematics returns {v1, v3, v2, a1, a3, a2}? 
            // WAIT - Standardize order. 
            // In your provided kinematics: output is {m1s, m2s, m3s, m1a, m2a, m3a}
            // Let's ensure we map index 4 to mod2 and 5 to mod3 correctly.
            // Kinematics provided: returns {mod1speed, mod2speed, mod3speed, mod1angle, mod2angle, mod3angle}
            // So:
            mod1power = output[0];
            mod2power = output[1]; 
            mod3power = output[2];
            
            mod1reference = output[3];
            mod2reference = output[4]; 
            mod3reference = output[5];
            
            initialized = true;
        }

        // 6. Efficient Turn & PID
        // Angle Wrap
        mod1P = mathsOperations.angleWrap(mod1P);
        mod2P = mathsOperations.angleWrap(mod2P);
        mod3P = mathsOperations.angleWrap(mod3P);
        
        mod1reference = mathsOperations.angleWrap(mod1reference);
        mod2reference = mathsOperations.angleWrap(mod2reference);
        mod3reference = mathsOperations.angleWrap(mod3reference);

        // Efficient Turn (Reverse power if angle > 90)
        double[] m1Eff = mathsOperations.efficientTurn(mod1reference, mod1P, mod1power);
        double[] m2Eff = mathsOperations.efficientTurn(mod2reference, mod2P, mod2power);
        double[] m3Eff = mathsOperations.efficientTurn(mod3reference, mod3P, mod3power);

        // PID Calculation
        double m1PID = mod1PID.pidOut(AngleUnit.normalizeDegrees(m1Eff[0] - mod1P));
        double m2PID = mod2PID.pidOut(AngleUnit.normalizeDegrees(m2Eff[0] - mod2P));
        double m3PID = mod3PID.pidOut(AngleUnit.normalizeDegrees(m3Eff[0] - mod3P));

        // 7. Differential Mixing (PID + Drive Power)
        // m1PID is rotation, m1Eff[1] is drive
        double[] m1Out = mathsOperations.diffyConvert(m1PID, -m1Eff[1]);
        double[] m2Out = mathsOperations.diffyConvert(m2PID, -m2Eff[1]); // Sign check: old code had different signs
        double[] m3Out = mathsOperations.diffyConvert(m3PID, -m3Eff[1]);
        
        // 8. Output
        mod1m1.setPower(m1Out[0]); mod1m2.setPower(m1Out[1]);
        mod2m1.setPower(m2Out[0]); mod2m2.setPower(m2Out[1]);
        mod3m1.setPower(m3Out[0]); mod3m2.setPower(m3Out[1]);

        // 9. Telemetry
        telemetry.addData("Heading", heading);
        telemetry.addData("M1 Angle/Ref", "%.1f / %.1f", mod1P, m1Eff[0]);
        telemetry.addData("M2 Angle/Ref", "%.1f / %.1f", mod2P, m2Eff[0]);
        telemetry.addData("M3 Angle/Ref", "%.1f / %.1f", mod3P, m3Eff[0]);
    }

    private double readEncoder(AnalogInput enc, double offset) {
        double raw = (enc.getVoltage() - 0.043) / 3.1 * 360.0;
        return AngleUnit.normalizeDegrees(raw - offset); // Minus offset to "Zero" it
    }

    private double getHeading(double polarity) {
        return AngleUnit.normalizeDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) * polarity - imuOffset);
    }
    
    public void resetIMU() { imuOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); }


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
        
        // Calculate Error based on the "TARGET_ANGLE" from Dashboard
        double[] m1Eff = mathsOperations.efficientTurn(targetAngle, mod1P, 0); // 0 power, just turning
        double[] m2Eff = mathsOperations.efficientTurn(targetAngle, mod2P, 0);
        double[] m3Eff = mathsOperations.efficientTurn(targetAngle, mod3P, 0);
        
        double m1PID = mod1PID.pidOut(AngleUnit.normalizeDegrees(m1Eff[0] - mod1P));
        double m2PID = mod2PID.pidOut(AngleUnit.normalizeDegrees(m2Eff[0] - mod2P));
        double m3PID = mod3PID.pidOut(AngleUnit.normalizeDegrees(m3Eff[0] - mod3P));
        
        // Apply ONLY rotation power (no drive)
        // Note: diffyConvert(rotate, translate)
        double[] m1Out = mathsOperations.diffyConvert(m1PID, 0);
        double[] m2Out = mathsOperations.diffyConvert(-m2PID, 0);
        double[] m3Out = mathsOperations.diffyConvert(-m3PID, 0);
        
        mod1m1.setPower(m1Out[0]); mod1m2.setPower(m1Out[1]);
        mod2m1.setPower(m2Out[0]); mod2m2.setPower(m2Out[1]);
        mod3m1.setPower(m3Out[0]); mod3m2.setPower(m3Out[1]);
        
        telemetry.addData("Target", targetAngle);
        telemetry.addData("M1 Pos", mod1P);
        telemetry.addData("M1 Error", m1Eff[0] - mod1P);
        telemetry.addData("M2 Pos", mod2P);
        telemetry.addData("M2 Error", m2Eff[0] - mod2P);
        telemetry.addData("M3 Pos", mod3P);
        telemetry.addData("M3 Error", m3Eff[0] - mod3P);

    }


}