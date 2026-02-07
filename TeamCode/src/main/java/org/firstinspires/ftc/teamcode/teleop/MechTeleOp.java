package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="TeleOP", group="Human Teleop")
public class MechTeleOp extends LinearOpMode{
    long lastLoopTime = 0;
    Robot robot;
    Pose2d goal = new Pose2d(0, 0);
    boolean telemetryOn = false;
    boolean telemetryOnTwo = false;

    double driveScale = SwerveTeleOpConfig.DRIVE_SPEED_SCALAR;
    double rotScale = SwerveTeleOpConfig.ROTATION_SPEED_SCALAR;


    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);

        robot.init();
        robot.updateGoal(goal);
        robot.updateTelemetry(telemetry);

        if(Turret.GOAL_Y == 0.0) {
            Turret.GOAL_Y = 144.0;
        }

        waitForStart();

        while(opModeIsActive()){
            double strafe = gamepad1.left_stick_x * driveScale;
            double forward = gamepad1.left_stick_y * driveScale;
            double rot = -gamepad1.right_stick_x * rotScale;
            robot.drive(strafe, forward, rot);

            if (gamepad1.right_trigger > 0.05) {
                robot.requestIntake();
            } else if (gamepad1.left_trigger > 0.05) {
                robot.requestOuttake();
            }

            if (gamepad1.left_bumper) {
                robot.requestSort();
            } else if (gamepad1.right_bumper) {
                robot.requestShot();
            }

            if (gamepad1.dpad_up){
                robot.requestIdle();
            }

            if (gamepad1.a) {
                robot.iamsacrificingmyfutureforthis();
            } else if (gamepad1.x) {
                robot.pleasekillmeiwannadie();
            } else if (gamepad1.y) {
                robot.youbetterflymeouttoworlds();
            }

            if (gamepad1.right_stick_button) {
                telemetryOn = !telemetryOnTwo;
            } else if (!gamepad1.right_stick_button){
                telemetryOnTwo = telemetryOn;
            }

            robot.update();
            long currentTime = System.currentTimeMillis();
            long loopTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
            telemetry.addData("Loop Time (ms)", loopTime);
            telemetry.addData("Frequency (Hz)", 1000.0 / loopTime);


            if (telemetryOn) {telemetry.addData("Status", robot.toString());}
            telemetry.update();
        }
    }
}
