package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config // This annotation is the magic key. It tells Dashboard to look here.
@TeleOp(name = "Dashboard Test", group = "Test")
public class DashboardTest extends OpMode {

    // These variables are "public static" so the Dashboard can see and edit them.
    // Use these for your PID coefficients later (kP, kI, kD).
    public static double TARGET_POSITION = 100;
    public static double TEST_SPEED = 0.5;

    @Override
    public void init() {
        // This lines sends telemetry to BOTH the driver station phone AND the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Print the variables to see them update in real-time
        telemetry.addData("Target Position", TARGET_POSITION);
        telemetry.addData("Test Speed", TEST_SPEED);

        // This is where you would normally run your PID calculation
        // double power = calculatePID(TARGET_POSITION, currentPosition);

        telemetry.update();
    }
}
