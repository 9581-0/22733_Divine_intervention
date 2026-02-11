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

        telemetry.addData("Status", "Initialized. Press Play to zero hood.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Send the command to move to position 0
            if (gamepad1.a) {hood.setPosition(0.0);}

            telemetry.addData("Servo Position", hood.getPosition());
            telemetry.addData("Status", "Running - Forced to 0");
            telemetry.update();
        }
    }
}