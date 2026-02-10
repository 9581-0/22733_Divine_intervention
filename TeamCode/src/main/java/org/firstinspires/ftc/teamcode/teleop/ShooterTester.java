package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Hood;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

@TeleOp
@Config
public class ShooterTester extends OpMode {
    DcMotorEx fM;

    public static double targetVelocity = 0.0;
    public static double currentVelocity = 0.0;

    public static double hoodPosition = 30.0;

    public static BasicFeedforwardParameters ff = new BasicFeedforwardParameters(1/2400.0, 0.0, 0.05);
    public static PIDCoefficients pids = new PIDCoefficients(0.00003, 0.0, 0.0);

    private final ControlSystem cS = ControlSystem.builder()
            .basicFF(ff)
            .velPid(pids)
            .build();
    Hood hood;

    @Override
    public void init() {
        fM = hardwareMap.get(DcMotorEx.class, "shooter");
        hood = new Hood(hardwareMap);
    }

    @Override
    public void loop() {
        cS.setGoal(new KineticState(0.0, targetVelocity));
        currentVelocity = fM.getVelocity();
        fM.setPower(cS.calculate(new KineticState(0.0, currentVelocity)));
        hood.setAngle(hoodPosition);
    }
}
