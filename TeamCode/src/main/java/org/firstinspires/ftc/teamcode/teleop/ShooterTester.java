package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Hood;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

@TeleOp
@Config
public class ShooterTester extends OpMode {
    private DcMotorEx fM;
    private Hood hood;

    private MultipleTelemetry t;

    public static double targetVelocity = 2280;
    public static double currentVelocity = 0.0;

    public static double hoodPosition = 30.0;

    // NextFTC feedforward + PID (NOT SDK PIDFCoefficients)
    public static BasicFeedforwardParameters ff =
            new BasicFeedforwardParameters(1.0 / 2400.0, 0.0, 0.05);

    public static PIDCoefficients velPid =
            new PIDCoefficients(0.00003, 0.0, 0.0);

    private final ControlSystem cS = ControlSystem.builder()
            .basicFF(ff)
            .velPid(velPid)
            .build();

    @Override
    public void init() {
        fM = hardwareMap.get(DcMotorEx.class, "shooter");
        fM.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = new Hood(hardwareMap);

        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Goal: velocity control
        cS.setGoal(new KineticState(0.0, targetVelocity));

        currentVelocity = fM.getVelocity();

        double power = cS.calculate(new KineticState(0.0, currentVelocity));
        power = clamp(power, -1.0, 1.0);

        fM.setPower(power);
        hood.setAngle(hoodPosition);

        t.addData("power", power);
        t.addData("targetVelocity", targetVelocity);
        t.addData("currentVelocity", currentVelocity);
        t.addData("hoodPosition", hoodPosition);
        t.update();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}


