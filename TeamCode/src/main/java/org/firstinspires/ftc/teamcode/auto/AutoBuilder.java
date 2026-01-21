package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.Pose2d;

public class AutoBuilder {
    // position constants go here
    private Robot robot;

    public AutoBuilder(HardwareMap map){
        robot = new Robot(map);
    }

    public void init(Pose2d goal, Telemetry telemetry) {
        robot.init();
        robot.updateGoal(goal);
        robot.updateTelemetry(telemetry);
    }

    public boolean driveToPosition(Pose2d pos){
        return robot.driveToPosition(pos);
    }

    public void changeState(String newState){
        switch (newState) {
            case "INTAKE":
                robot.requestIntake();
                break;
            case "SORT":
                robot.requestSort();
                break;
            case "SHOOT":
                robot.requestShot();
                break;
            case "OUTTAKE":
                robot.requestOuttake();
                break;
            case "IDLE":
                robot.requestIdle();
                break;
            default:
                break;
        }
    }
}
