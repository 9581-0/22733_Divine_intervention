package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="TEST", group="Human Teleop")
public class TestTele extends LinearOpMode{
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

        Turret.GOAL_Y = 72.0;
        Turret.GOAL_X = -72.0;

        robot.init();
        robot.setPose(Robot.pose);
        robot.updateGoal(goal);
        robot.updateTelemetry(telemetry);

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

            if (gamepad1.leftBumperWasPressed()) {
                robot.requestSort();
            }
            else if (gamepad1.rightBumperWasPressed()) {
                robot.requestShot();
            }

            if (gamepad1.dpadUpWasPressed()){
                robot.requestIdle();
            }

            if (gamepad1.aWasPressed()) {
                robot.iamsacrificingmyfutureforthis();
            }
            else if (gamepad1.xWasPressed()) {
                robot.pleasekillmeiwannadie();
            }
            else if (gamepad1.yWasPressed()) {
                robot.youbetterflymeouttoworlds();
            }

            if (gamepad1.right_stick_button) {
                telemetryOn = !telemetryOnTwo;
            } else if (!gamepad1.right_stick_button){
                telemetryOnTwo = telemetryOn;
            }

            if(gamepad1.leftStickButtonWasPressed()) {
                Turret.tracking = false;
            }

            if(gamepad1.startWasPressed()) {
                robot.setPose(new Pose2d(72.0, 72.0, Math.PI));
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
