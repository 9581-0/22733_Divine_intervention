package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Zero Hood Servo", group = "Setup")
public class HoodOp extends LinearOpMode {

    private Servo hood;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        // The string "hood" must match the name in your Robot Configuration.
        hood = hardwareMap.get(Servo.class, "hood");
double pos = 0.4;
        waitForStart();
        while (opModeIsActive()) {
            hood.setPosition(pos);

            if (gamepad1.dpad_up) {pos += 0.001;}
            if (gamepad1.dpad_down) {pos -= 0.001;}
            telemetry.addData("Servo Position", 360* (hood.getPosition()));
            telemetry.addData("Status", "Running - Forced to 0");
            telemetry.update();

        }
    }
}