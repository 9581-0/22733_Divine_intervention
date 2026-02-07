package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public abstract class  DecodePedroAutoBase extends LinearOpMode {
    private static final double SHOOT_DURATION_SEC = 2.0;

    protected abstract DecodePedroAutoPlan.Alliance getAlliance();

    protected abstract DecodePedroAutoPlan.StartSide getStartSide();

    private enum AutoStage {
        INTAKE_SET1,
        SHOOT_SET1,
        INTAKE_SET2,
        SHOOT_SET2,
        DONE
    }

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        DecodePedroAutoPlan.Route route = DecodePedroAutoPlan.build(getAlliance(), getStartSide());
        ElapsedTime stageTimer = new ElapsedTime();
        AutoStage stage = AutoStage.INTAKE_SET1;
        AutoStage lastStage = null;

        robot.init();
        robot.updateGoal(route.goalPose);
        robot.setPose(route.startPose);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            robot.update();

            if (stage != lastStage) {
                switch (stage) {
                    case INTAKE_SET1:
                    case INTAKE_SET2:
                        robot.requestIntake();
                        break;
                    case SHOOT_SET1:
                    case SHOOT_SET2:
                        robot.requestShot();
                        stageTimer.reset();
                        break;
                    case DONE:
                        robot.requestIdle();
                        break;
                    default:
                        break;
                }
                lastStage = stage;
            }

            switch (stage) {
                case INTAKE_SET1:
                    if (route.intakePathSet1.advanceIfReached(
                            robot.driveToPosition(route.intakePathSet1.getTarget()))) {
                        stage = AutoStage.SHOOT_SET1;
                    }
                    break;
                case SHOOT_SET1:
                    robot.driveToPosition(route.shootPose);
                    if (stageTimer.seconds() >= SHOOT_DURATION_SEC) {
                        stage = AutoStage.INTAKE_SET2;
                    }
                    break;
                case INTAKE_SET2:
                    if (route.intakePathSet2.advanceIfReached(
                            robot.driveToPosition(route.intakePathSet2.getTarget()))) {
                        stage = AutoStage.SHOOT_SET2;
                    }
                    break;
                case SHOOT_SET2:
                    robot.driveToPosition(route.shootPose);
                    if (stageTimer.seconds() >= SHOOT_DURATION_SEC) {
                        stage = AutoStage.DONE;
                    }
                    break;
                case DONE:
                    robot.drive(0, 0, 0);
                    break;
                default:
                    break;
            }

            telemetry.addData("Stage", stage);
            telemetry.addData("StartSide", getStartSide());
            telemetry.addData("Alliance", getAlliance());
            telemetry.update();
        }
    }
}
