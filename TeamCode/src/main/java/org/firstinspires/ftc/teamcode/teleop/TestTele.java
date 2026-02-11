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

        Turret.GOAL_Y = 142.0;
        Turret.GOAL_X = 142.0;

        robot.init();
        robot.setPose(new Pose2d(144.0, 0.0, 90.0));

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

            if (gamepad1.left_bumper) {
                robot.requestSort();
            }
            else if (gamepad1.rightBumperWasPressed()) {
                robot.requestShot();
            }

            if (gamepad1.dpad_up){
                robot.requestIdle();
            }

            if (gamepad1.a) {
                robot.iamsacrificingmyfutureforthis();
            }
            else if (gamepad1.x) {
                robot.pleasekillmeiwannadie();
            }
            else if (gamepad1.y) {
                robot.youbetterflymeouttoworlds();
            }

            if (gamepad1.right_stick_button) {
                telemetryOn = !telemetryOnTwo;
            } else if (!gamepad1.right_stick_button){
                telemetryOnTwo = telemetryOn;
            }

            if(gamepad1.left_stick_button) {
                Turret.tracking = !Turret.tracking;
            }

            if(gamepad1.start) {
                robot.setPose(new Pose2d(72.0, 72.0, 0.0));
            }

            robot.update();
            long currentTime = System.currentTimeMillis();
            long loopTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
            telemetry.addData("Loop Time (ms)", loopTime);
            telemetry.addData("Frequency (Hz)", 1000.0 / loopTime);


            if (true) {// telemetry.addData("Status", robot.toString());
            telemetry.addData("tracking", Turret.tracking);
            telemetry.addData("robot pinpoint pose", robot.odo.getPosition());
            telemetry.addData("pos", robot.turret.getPosition());
            telemetry.addData("target", robot.turret.target);
            telemetry.addData("goal x", Turret.GOAL_X);
            telemetry.addData("goal y", Turret.GOAL_Y);
            }
            telemetry.update();
        }
    }
}
