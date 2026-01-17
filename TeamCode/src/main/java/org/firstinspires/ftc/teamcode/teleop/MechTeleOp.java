package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="Mech Teleop", group="Human Teleop")
public class MechTeleOp extends LinearOpMode{
    Robot robot;
    Pose2d goal;

    private static final double HOOD_STEP = 0.02;
    private static final double TURRET_STEP = 5.0;
    private boolean leftTriggerLatch = false;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);

        robot.init();
        robot.updateGoal(goal);
        robot.updateTelemetry(telemetry);

        waitForStart();

        while(opModeIsActive()){
            double strafe = gamepad1.left_stick_x;
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
                telemetry.addData("trigger being held", "yes");
            } else {
                robot.setIntakePower(0);
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

            robot.update();

            telemetry.addData("Status", robot.toString());
            telemetry.update();
        }
    }
}
