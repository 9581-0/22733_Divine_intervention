package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

@Autonomous(name = "Simple Drive Off Auto", group = "Auton")
public class SimpleDriveOffAuto extends LinearOpMode {
    private static final double DRIVE_SPEED = 0.5;
    private static final double DRIVE_DURATION_SEC = 3.5;

    @Override
    public void runOpMode() {
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        timer.reset();
        while (opModeIsActive() && timer.seconds() < DRIVE_DURATION_SEC) {
            swerve.driveWithConfig(0, DRIVE_SPEED, 0);
            telemetry.addData("Status", "Driving");
            telemetry.update();
        }

        swerve.driveWithConfig(0, 0, 0);
        telemetry.addData("Status", "Complete");
        telemetry.update();
    }
}
