package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utility.Encoder;

@TeleOp(name = "SwerveTeleOp", group = "Linear Opmode")
public class SwerveTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetryReporter telemetryReporter = new TelemetryReporter(telemetry);

        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap, true, false,
                false);
        DriveController driveController = new DriveController(swerve);

        // Turret turret = new Turret(hardwareMap);
        // Indexer indexer = new Indexer(hardwareMap);
        // ManipulatorController manipulatorController = new ManipulatorController(turret, indexer);

        // Encoder leftOdo = new Encoder(hardwareMap, "leftOdo");
        // Encoder rightOdo = new Encoder(hardwareMap, "rightOdo");
        // LocalizationManager localizationManager = new LocalizationManager(leftOdo, rightOdo, swerve::getHeading);

        telemetryReporter.addInitializationStatus();

        waitForStart();
        // localizationManager.reset();

        while (opModeIsActive()) {
            DriveController.DriveState driveState = driveController.driveWithGamepad(gamepad1);
            // ManipulatorController.ManipulatorState manipulatorState = manipulatorController.updateFromGamepad(gamepad2);
            // Pose2d pose = localizationManager.updateAndGetPose();
            Pose2d pose = new Pose2d();

            telemetryReporter.reportLoop(gamepad1, pose, driveState, null);
        }
    }
}
