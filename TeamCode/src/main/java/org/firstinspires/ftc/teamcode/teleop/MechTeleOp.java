package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="Mech Teleop", group="Human Teleop")
public class MechTeleOp extends LinearOpMode{
    Robot robot;
    Pose2D goal;

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

            telemetry.addData("Status", robot.toString());
            telemetry.update();
        }
    }
}
