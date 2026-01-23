package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="A Shrimp made this teleop", group="Human Teleop")
public class MechTeleOp extends LinearOpMode{
    long lastLoopTime = 0;
    Robot robot;
    Pose2d goal = new Pose2d(0, 0);

    private static final double HOOD_STEP = 0.02;
    private static final double TURRET_STEP = 5.0;
    // TUNE: Update these corner coordinates using the official FTC field spec for your season.
    private static final double FIELD_HALF_SIZE_INCHES = 72.0;
    private static final Pose2d RED_CORNER_GOAL = new Pose2d(FIELD_HALF_SIZE_INCHES, FIELD_HALF_SIZE_INCHES, 0);
    private static final Pose2d BLUE_CORNER_GOAL = new Pose2d(-FIELD_HALF_SIZE_INCHES, -FIELD_HALF_SIZE_INCHES, 0);
    private boolean leftTriggerLatch = false;
    private boolean goalSelectLatch = false;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);

        robot.init();
        goal = RED_CORNER_GOAL;
        robot.updateGoal(goal);
        robot.updateTelemetry(telemetry);

        waitForStart();

        while(opModeIsActive()){
            double strafe = -gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double rot = gamepad1.right_stick_x;
            robot.drive(strafe, forward, rot);

            if (gamepad1.left_trigger > 0.5) {
                if (!leftTriggerLatch) {
                    robot.toggleFlywheel();
                    leftTriggerLatch = true;
                }
            } else {
                leftTriggerLatch = false;
            }

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

            if (gamepad1.dpad_up) {
                robot.adjustHoodPosition(HOOD_STEP);
            } else if (gamepad1.dpad_down) {
                robot.adjustHoodPosition(-HOOD_STEP);
            }

            if (gamepad1.dpad_right) {
                robot.adjustTurretAngle(TURRET_STEP);
            } else if (gamepad1.dpad_left) {
                robot.adjustTurretAngle(-TURRET_STEP);
            }

            if (!goalSelectLatch) {
                if (gamepad1.a) {
                    goal = RED_CORNER_GOAL;
                    robot.updateGoal(goal);
                    goalSelectLatch = true;
                } else if (gamepad1.b) {
                    goal = BLUE_CORNER_GOAL;
                    robot.updateGoal(goal);
                    goalSelectLatch = true;
                }
            } else if (!gamepad1.a && !gamepad1.b) {
                goalSelectLatch = false;
            }

            if (gamepad1.a) {
                robot.iamsacrificingmyfutureforthis();
            } else if (gamepad1.x) {
                robot.pleasekillmeiwannadie();
            } else if (gamepad1.y) {
                robot.youbetterflymeouttoworlds();
            }

            robot.update();
            long currentTime = System.currentTimeMillis();
            long loopTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;
            telemetry.addData("Loop Time (ms)", loopTime);
            telemetry.addData("Frequency (Hz)", 1000.0 / loopTime);


            telemetry.addData("Status", robot.toString());
            telemetry.addData("Auto-aim goal", goal);
            telemetry.update();
        }
    }
}
