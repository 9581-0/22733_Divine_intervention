package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
 
import org.firstinspires.ftc.teamcode.util.statemachine.State;
import org.firstinspires.ftc.teamcode.util.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.statemachine.Transition;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;

public class Robot {
    private StateMachine state;

    private Shooter shooter;
    private Spindexer spindex;
    private Intake intake;
    private SwerveDrive swerve;
    private GoBildaPinpointDriver odo;

    private boolean requestIntake = false, requestOuttake = false, requestShot = false, requestSort = false, requestIdle = false;
    private Pose2d pose, goal;
    private Telemetry telemetry;

    public Robot (HardwareMap map) {
        shooter = new Shooter(map);
        spindex = new Spindexer(map);
        intake = new Intake(map);
        swerve = new SwerveDrive(telemetry, map);

        odo = map.get(GoBildaPinpointDriver.class, "odo");

        pose = new Pose2d(0, 0, Math.toRadians(0));
        goal = new Pose2d(0, 0, Math.toRadians(0));
  
        State[] states = createStates();
        state = new StateMachine(states);
    }

    private State[] createStates(){
        State[] states = new State(5);

        states[0] = new State("IDLE")
                .setEntry(() -> {
                    requestIdle = false;
                    intake.setPower(0);
                })
                .addTransition(new Transition(() -> requestIntake, "INTAKE"))
                .addTransition(new Transition(() -> requestOuttake, "OUTTAKE"))
                .addTransition(new Transition(() -> requestSort, "SORT"))
                .addTransition(new Transition(() -> requestShot, "SHOOT"));
        
        states[1] = new State("INTAKE")
                .setEntry(() -> {
                    requestIntake = false;
                    intake.setPower(1);
                })
                .setFallbackState("IDLE")
                .addTransition(new Transition(() -> spindex.isFull(), "IDLE"))
                .addTransition(new Transition(() -> requestIdle, "IDLE"))
                .addTransition(new Transition(() -> requestOuttake, "OUTTAKE"))
                .addTransition(new Transition(() -> requestSort, "SORT"))
                .addTransition(new Transition(() -> requestShot, "SHOOT"));
        
        states[2] = new State("OUTTAKE")
                .setEntry(() -> {
                    requestOuttake = false;
                    intake.setPower(-1);
                })
                .setFallbackState("IDLE")
                .addTransition(new Transition(() -> requestIdle, "IDLE"))
                .addTransition(new Transition(() -> requestIntake, "INTAKE"))
                .addTransition(new Transition(() -> requestSort, "SORT"))
                .addTransition(new Transition(() -> requestShot, "SHOOT"));
        
        states[3] = new State("SORT")
                .setEntry(() -> {
                    requestSort = false;
                })
                .setFallbackState("IDLE")
                .addTransition(new Transition(() -> requestIdle, "IDLE"))
                .addTransition(new Transition(() -> spindex.isIdle(), "SHOOT"));
        
        states[4] = new State("SHOOT")
                .setEntry(() -> {
                    requestShot = false;
                    shooter.updatePose(pose);
                    shooter.requestShot();
                    shooter.updateGoal(goal);
                    spindex.shoot();
                })
                .setDuring(() -> {
                    shooter.updatePose(pos);
                })
                .setExit(() -> {
                    shooter.requestIdle();
                })
                .setFallbackState("IDLE")
                .addTransition(new Transition(() -> spindex.isEmpty(), "IDLE"))
                .addTransition(new Transition(() -> requestIdle, "IDLE"));
    }

    public void init(){
        state.start();
    }

    public void update(){
        shooter.update();
        spindex.update();
        odo.update();

        pose = odo.getPosition();
    }

    public void drive(double strafe, double forward, double rot){
        swerve.driveWithConfig(strafe, forward, rot);
    }

    public void updateGoal(Pose2D goale) {
        goal = goale;
        shooter.updateGoal(goal);
    }

    public void updateTelemetry(Telemetry telem) {
        telemetry = telem;
        swerve.telemetry = telem;
    }

    public void requestIdle(){requestIdle = true;}
    public void requestIntake(){requestIntake = true;}
    public void requestOuttake(){requestOuttake = true;}
    public void requestSort(){requestSort = true;}
    public void requestShot(){requestShot = true;}

    @Override
    public String toString(){
        return "Robot {" + 
                intake.toString() + 
                spindex.toString() + 
                shooter.toString() + 
                "pos = " + pose + "}";
    }
}
