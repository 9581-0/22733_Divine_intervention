package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

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

    final private myDcMotorEx mod1m1, mod1m2, mod2m1, mod2m2, mod3m1, mod3m2;
    final private AnalogInput mod1E, mod2E, mod3E;
    final private Telemetry telemetry;
    final private boolean eff;
    private final boolean useImu;
    private final boolean fieldCentric;
    private boolean initializedReferences = false;
    private double module1Adjust = 337, module2Adjust = 285, module3Adjust = 0;
    private final PIDcontroller mod1PID = new PIDcontroller(0.1, 0.002, 3, 1, 0.5);
    private final PIDcontroller mod2PID = new PIDcontroller(0.1, 0.002, 2, 0.5, 0.5);
    private final PIDcontroller mod3PID = new PIDcontroller(0.1, 0.002, 1, 0.5, 0.75);
    private final swerveKinematics swavemath = new swerveKinematics();

    double mod1reference = 0;
    double mod2reference = 0;
    double mod3reference = 0;
    double heading;
    private double imuOffset = 0;

    public SwerveDrive(Telemetry telemetry, HardwareMap hardwareMap, boolean eff, boolean useImu,
            boolean fieldCentric) {
        mod1m1 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod1m1"));
        mod1m2 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod1m2"));
        mod2m1 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod2m1"));
        mod2m2 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod2m2"));
        mod3m1 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod3m1"));
        mod3m2 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod3m2"));
        mod1E = hardwareMap.get(AnalogInput.class, "mod1E");
        mod2E = hardwareMap.get(AnalogInput.class, "mod2E");
        mod3E = hardwareMap.get(AnalogInput.class, "mod3E");

        mod3m1.setDirection(DcMotorSimple.Direction.REVERSE);

        for (myDcMotorEx motor : new myDcMotorEx[] { mod1m1, mod1m2, mod2m1, mod2m2, mod3m1, mod3m2 }) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        mod3m1.setPowerThresholds(0.05, 0);
        mod3m2.setPowerThresholds(0.05, 0);
        mod1m1.setPowerThresholds(0.05, 0);
        mod1m2.setPowerThresholds(0.05, 0);
        mod2m1.setPowerThresholds(0.05, 0);
        mod2m2.setPowerThresholds(0.05, 0);

        mod2m2.setDirection(DcMotorSimple.Direction.REVERSE);
        mod3m2.setDirection(DcMotorSimple.Direction.REVERSE);

        this.useImu = useImu;
        this.fieldCentric = fieldCentric;
        if (useImu) {
            IMU foundImu = hardwareMap.tryGet(IMU.class, "pinpoint");
            if (foundImu == null) {
                foundImu = hardwareMap.tryGet(IMU.class, "imu");
            }

            if (foundImu != null) {
                IMU.Parameters localParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
                foundImu.initialize(localParams);
            }
            imu = foundImu;
        } else {
            imu = null;
        }
        // Store telemetry and efficiency flag
        this.telemetry = telemetry;
        this.eff = eff;
    }

    /**
     * Drive using explicit x, y, and rotation values.
     */
    public void drive(double strafe, double forward, double rot) {
        double mod1P = readEncoderDegrees(mod1E, module1Adjust);
        double mod2P = readEncoderDegrees(mod2E, module2Adjust);
        double mod3P = readEncoderDegrees(mod3E, module3Adjust);

        // Prevent tiny encoder noise from commanding a correction when sticks are idle.
        boolean sticksActive = forward != 0 || strafe != 0 || rot != 0;
        final double steerDeadbandDeg = 1.0;

        if (!initializedReferences) {
            mod1reference = mod1P;
            mod2reference = mod2P;
            mod3reference = mod3P;
            initializedReferences = true;
        }

        // Update heading of robot
        heading = useImu && imu != null ? getHeadingInDegrees() : 0;

        // Retrieve the angle and power for each module
        double[] output = swavemath.calculate(forward, strafe, rot, heading, fieldCentric);
        double mod1power = output[0];
        double mod3power = output[1];
        double mod2power = output[2];

        // keep previous module heading if joystick not being used
        if (forward != 0 || strafe != 0 || rot != 0) {
            mod1reference = output[3];
            mod3reference = output[5];
            mod2reference = output[4];
        }

        // Anglewrap all the angles so that the module turns both ways
        mod1P = mathsOperations.angleWrap(mod1P);
        mod2P = mathsOperations.angleWrap(mod2P);
        mod3P = mathsOperations.angleWrap(mod3P);

        mod1reference = mathsOperations.angleWrap(mod1reference);
        mod2reference = mathsOperations.angleWrap(mod2reference);
        mod3reference = mathsOperations.angleWrap(mod3reference);

        // Make sure that a module never turns more than 90 degrees
        double[] mod1efvalues = mathsOperations.efficientTurn(mod1reference, mod1P, mod1power);

        double[] mod2efvalues = mathsOperations.efficientTurn(mod2reference, mod2P, mod2power);

        double[] mod3efvalues = mathsOperations.efficientTurn(mod3reference, mod3P, mod3power);

        if (eff) {
            mod1reference = mod1efvalues[0];
            mod1power = mod1efvalues[1];
            mod2reference = mod2efvalues[0];
            mod2power = mod2efvalues[1];
            mod3reference = mod3efvalues[0];
            mod3power = mod3efvalues[1];
        }

        // change coax values into diffy values from pid and power
        double mod1Error = AngleUnit.normalizeDegrees(mod1reference - mod1P);
        double mod2Error = AngleUnit.normalizeDegrees(mod2reference - mod2P);
        double mod3Error = AngleUnit.normalizeDegrees(mod3reference - mod3P);

        double mod1Steer = Math.abs(mod1Error) < steerDeadbandDeg && !sticksActive ? 0 : mod1PID.pidOut(mod1Error);
        double mod2Steer = Math.abs(mod2Error) < steerDeadbandDeg && !sticksActive ? 0 : -mod2PID.pidOut(mod2Error);
        double mod3Steer = Math.abs(mod3Error) < steerDeadbandDeg && !sticksActive ? 0 : -mod3PID.pidOut(mod3Error);

        double[] mod1values = mathsOperations.diffyConvert(mod1Steer, -mod1power);
        mod1m1.setPower(mod1values[0]);
        mod1m2.setPower(mod1values[1]);
        double[] mod2values = mathsOperations.diffyConvert(mod2Steer, mod2power);
        mod2m1.setPower(mod2values[0]);
        mod2m2.setPower(mod2values[1]);
        double[] mod3values = mathsOperations.diffyConvert(mod3Steer, mod3power);
        mod3m1.setPower(mod3values[0]);
        mod3m2.setPower(mod3values[1]);

        telemetry.addData("mod1reference", mod1reference);
        telemetry.addData("mod2reference", mod2reference);
        telemetry.addData("mod3reference", mod3reference);

        telemetry.addData("mod1P", mod1P);
        telemetry.addData("mod2P", mod2P);
        telemetry.addData("mod3P", mod3P);
        telemetry.addData("mod1Error", mod1Error);
        telemetry.addData("mod2Error", mod2Error);
        telemetry.addData("mod3Error", mod3Error);
        telemetry.addData("mod1Steer", mod1Steer);
        telemetry.addData("mod2Steer", mod2Steer);
        telemetry.addData("mod3Steer", mod3Steer);
    }

    /**
     * Rotate robot by a given angle (degrees).
     */
    public void rotateKids(double angle) {
        imuOffset = AngleUnit.normalizeDegrees(imuOffset + angle);
    }

    /**
     * Reset IMU yaw to zero.
     */
    public void resetIMU() {
        imuOffset = 0;
        if (imu != null) {
            imu.resetYaw();
        }
    }

    // Tune module PIDs
    /**
     * Set PID coefficients for module 1 (others share same PID instance).
     */
    public void setPIDCoeffs(double Kp, double Kd, double Ki, double Kf, double limit) {
        mod1PID.setPIDgains(Kp, Kd, Ki, Kf, limit);
    }

    // Tunable module zeroing
    public void setModuleAdjustments(double module1Adjust, double module2Adjust, double module3Adjust) {
        this.module1Adjust = module1Adjust;
        this.module2Adjust = module2Adjust;
        this.module3Adjust = module3Adjust;
    }

    /**
     * Get robot heading in degrees.
     */
    public double getHeading() {
        return getHeadingInDegrees();
    }

    /**
     * Convert encoder voltage to degrees with offset.
     */
    private double readEncoderDegrees(AnalogInput encoder, double offsetDegrees) {
        return AngleUnit.normalizeDegrees((encoder.getVoltage() - 0.043) / 3.1 * 360 + offsetDegrees);
    }

    /**
     * Internal helper to get heading from IMU.
     */
    private double getHeadingInDegrees() {
        if (imu != null) {
            return AngleUnit.normalizeDegrees(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - imuOffset);
        }
        return 0;
    }

    /**
     * Drive using a Gamepad. Left stick controls translation, right stick X
     * controls rotation.
     */
    public void drive(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y; // invert Y for forward
        double rot = gamepad.right_stick_x;
        drive(x, y, rot);
    }
}
