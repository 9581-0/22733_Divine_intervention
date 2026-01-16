package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

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





        while (opModeIsActive()) {
            
            // 1. Inputs
            double driveScale = SwerveTeleOpConfig.DRIVE_SPEED_SCALAR;
            double rotScale = SwerveTeleOpConfig.ROTATION_SPEED_SCALAR;
            
            double strafe = gamepad1.left_stick_x * driveScale;
            double forward = gamepad1.left_stick_y * driveScale;

            double botheadingX = gamepad1.right_stick_x;
            double hotheadingY = gamepad1.right_stick_y;

            double rot = gamepad1.right_stick_x * rotScale;

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
                SwerveTeleOpConfig.FIELD_CENTRIC,
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