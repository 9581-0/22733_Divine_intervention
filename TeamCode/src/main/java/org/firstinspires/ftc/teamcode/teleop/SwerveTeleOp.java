package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Swerve TeleOp (Final)", group = "Main")
public class SwerveTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Setup Telemetry (Phone + Dashboard)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Init Subsystems
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);
        // Shooting Status



        telemetry.addData("Status", "Ready. Run 'Module Zeroing' if wheels are not aligned.");
        telemetry.update();

        waitForStart();

        double targetHeading = swerve.getHeadingDeg(SwerveTeleOpConfig.IMU_POLARITY);
        boolean fieldCentric = SwerveTeleOpConfig.FIELD_CENTRIC;
        boolean fieldCentricLatch = false;
        double headingIntegral = 0;
        double lastHeadingError = 0;
        ElapsedTime headingTimer = new ElapsedTime();
        headingTimer.reset();




        while (opModeIsActive()) {
            
            // 1. Inputs
            double driveScale = SwerveTeleOpConfig.DRIVE_SPEED_SCALAR;
            double rotScale = SwerveTeleOpConfig.ROTATION_SPEED_SCALAR;
            
            double strafe = gamepad1.left_stick_x * driveScale;
            double forward = -gamepad1.left_stick_y * driveScale;

            double headingStickX = gamepad1.right_stick_x;
            double headingStickY = -gamepad1.right_stick_y;

            double rot = headingStickX * rotScale;
            if (SwerveTeleOpConfig.HEADING_HOLD_ENABLED) {
                double headingMagnitude = Math.hypot(headingStickX, headingStickY);
                if (headingMagnitude > SwerveTeleOpConfig.HEADING_STICK_DEADBAND) {
                    targetHeading = Math.toDegrees(Math.atan2(headingStickX, headingStickY));
                    headingIntegral = 0;
                }
                double currentHeading = swerve.getHeadingDeg(SwerveTeleOpConfig.IMU_POLARITY);
                double headingError = AngleUnit.normalizeDegrees(targetHeading - currentHeading);
                double dt = Math.max(headingTimer.seconds(), 1e-3);
                headingTimer.reset();
                headingIntegral += headingError * dt;
                headingIntegral = Math.max(-SwerveTeleOpConfig.HEADING_HOLD_INTEGRAL_LIMIT,
                    Math.min(SwerveTeleOpConfig.HEADING_HOLD_INTEGRAL_LIMIT, headingIntegral));
                double headingDerivative = (headingError - lastHeadingError) / dt;
                lastHeadingError = headingError;
                double pidOutput =
                    (headingError * SwerveTeleOpConfig.HEADING_HOLD_KP) +
                    (headingIntegral * SwerveTeleOpConfig.HEADING_HOLD_KI) +
                    (headingDerivative * SwerveTeleOpConfig.HEADING_HOLD_KD);
                rot = Math.max(-1.0, Math.min(1.0, pidOutput)) * rotScale;
            }

            if (!fieldCentricLatch && gamepad1.y) {
                fieldCentric = !fieldCentric;
                fieldCentricLatch = true;
            } else if (!gamepad1.y) {
                fieldCentricLatch = false;
            }

            // 2. Drive Command 
            // We pass ALL config values here so they update live!
            swerve.drive(
                strafe, forward, rot,
                SwerveTeleOpConfig.module1Adjust,
                SwerveTeleOpConfig.module2Adjust,
                SwerveTeleOpConfig.module3Adjust,
                SwerveTeleOpConfig.Kp,
                SwerveTeleOpConfig.Kd,
                SwerveTeleOpConfig.Ki,
                SwerveTeleOpConfig.Kf,
                SwerveTeleOpConfig.Kl,
                fieldCentric,
                SwerveTeleOpConfig.IMU_POLARITY,
                SwerveTeleOpConfig.ROBOT_RADIUS
            );

            // 3. Reset IMU
            if (gamepad1.options || gamepad1.start) {
                swerve.resetIMU();
                gamepad1.rumble(500);
            }

            //shooter








            // 4. Update Telemetry
            telemetry.update();
        }
    }
}
