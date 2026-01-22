package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="A Shrimp made this teleop", group="Human Teleop")
public class MechTeleOp extends LinearOpMode{
    Robot robot;
    Pose2d goal = new Pose2d(0, 0);

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);

        robot.init();
        robot.updateGoal(goal);
        robot.updateTelemetry(telemetry);

        waitForStart();

        while(opModeIsActive()){
            double strafe = -gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double rot = gamepad1.right_stick_x;
            robot.drive(strafe, forward, rot);

            if (gamepad1.right_trigger > 0.05) {
                robot.requestIntake();
                telemetry.addData("trigger being held", "yes");
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

            robot.update();

            telemetry.addData("Status", robot.toString());
            telemetry.update();
        }
    }
}
