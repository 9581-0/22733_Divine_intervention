package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

import org.firstinspires.ftc.teamcode.util.statemachine.State;
import org.firstinspires.ftc.teamcode.util.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.statemachine.Transition;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;

import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;

public class Robot {
    private final StateMachine state;

    private final Shooter shooter;
    private final Spindexer spindex;
    private final Intake intake;
    private final SwerveDrive swerve;
    private final GoBildaPinpointDriver odo;

    private boolean requestIntake = false, requestOuttake = false, requestShot = false, requestSort = false, requestIdle = false;
    private Pose2d pose, goal;
    private Telemetry telemetry;

    public static double tP = 0.005, tD = 0.0005;
    private PIDF tpid = new PIDF(tP, tD);
    public static double rP = 0.01, rD = 0.001;
    private PIDF rpid = new PIDF(rP, rD);

    private List<LynxModule> allHubs;

    public Robot (HardwareMap map) {
        shooter = new Shooter(map);
        spindex = new Spindexer(map);
        intake = new Intake(map);
        swerve = new SwerveDrive(telemetry, map);

        odo = map.get(GoBildaPinpointDriver.class, "odo");

        pose = new Pose2d(0, 0, Math.toRadians(0));
        goal = new Pose2d(0, 0, Math.toRadians(0));

        tpid.setTolerance(20);
        rpid.setTolerance(3);

        allHubs = map.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
  
        State[] states = createStates();
        state = new StateMachine(states);
    }

    private State[] createStates(){
        State[] states = new State[5];

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
                    shooter.updatePose(pose);
                })
                .setExit(() -> {
                    shooter.requestIdle();
                })
                .setFallbackState("IDLE")
                .addTransition(new Transition(() -> spindex.isEmpty(), "IDLE"))
                .addTransition(new Transition(() -> requestIdle, "IDLE"));
    
        return states;
    }

    public void init(){
        state.start();
        shooter.init();
    }

    public void update(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        state.run();

        shooter.update();
        spindex.update();
        intake.update();
        odo.update();

        pose = odo.getPosition();
    }

    public void drive(double strafe, double forward, double rot){
        swerve.driveWithConfig(strafe, forward, rot);
    }

    public boolean driveToPosition(Pose2d pos){
        drive(
            tpid.calculate(pose.x, pos.x), 
            tpid.calculate(pose.y, pos.y), 
            rpid.calculate(pose.heading, pos.heading));
        return tpid.atSetPoint() && rpid.atSetPoint();
    }

    public void updateGoal(Pose2d goale) {
        goal = goale;
        shooter.updateGoal(goal);
    }

    public void updateTelemetry(Telemetry telem) {
        telemetry = telem;
        swerve.updateTelemetry(telem);
    }

    public void enableSort(){spindex.enableSort();}
    public void disableSort(){spindex.disableSort();}

    public void requestIdle(){
        requestIdle = true;
    }
    public void requestIntake(){
        requestIntake = true;
        intake.setPower(1);

    }
    public void requestOuttake(){
        requestOuttake = true;

    }
    public void requestSort(){
        spindex.enableSort();
        requestSort = true;
    }
    public void requestShot(){
        spindex.disableSort();
        requestShot = true;
    }

    public void pleasekillmeiwannadie(){spindex.pleasekillmeiwannadie();}
    public void youbetterflymeouttoworlds(){spindex.youbetterflymeouttoworlds();}
    public void iamsacrificingmyfutureforthis(){spindex.iamsacrificingmyfutureforthis();}

    @Override
    public String toString(){
        return "Robot {" + 
                intake.toString() + 
                spindex.toString() +
                shooter.toString() + 
                "pos = " + pose + "}";
    }
}
