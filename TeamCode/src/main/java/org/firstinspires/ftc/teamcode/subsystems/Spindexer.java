package org.firstinspires.ftc.teamcode.subsystems;

import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.wrappers.Sensorange;
import org.firstinspires.ftc.teamcode.util.statemachine.State;
import org.firstinspires.ftc.teamcode.util.statemachine.StateMachine;

public class Spindexer {
    private CRServo LServo;
    private CRServo RServo;

    private ColorSensor colora;
    private ColorSensor colorb;
    private ColorSensor colorc;
    private Sensorange encoder;

    private StateMachine state;
    public static double P = 0.05, D = 0.001;
    private PIDF pid = new PIDF(P, D);
    private double target;

    private ArrayList<String> motif = new ArrayList<>();
    private ArrayList<String> stored = new ArrayList<>();
    private boolean requestFire, requestSort, requestIdle = false;
    private boolean sort = false;
    private String status = "NO, THE THING IS NOT RUNNING";

    public Spindexer(HardwareMap map){
        LServo = map.get(CRServo.class, "IndexServoL");
        RServo = map.get(CRServo.class, "IndexServoR");

        encoder = new Sensorange("encoder", map);

        colora = map.get(ColorSensor.class, "color1");
        colorb = map.get(ColorSensor.class, "color2");
        colorc = map.get(ColorSensor.class, "color3");

        pid.setTolerance(3);

        scanDexer();

        // State[] states = createStates();
        // state = new StateMachine(states);
    }

    // private State[] createStates(){
    //     State[] states = new State[3];

    //     states[0] = new State("IDLE")
    //         .setEntry(() -> {
    //             requestIdle = false;
    //         })
    //         .setDuring(() -> {
    //             scanDexer();
    //             runPID();
    //         })
    //         .addTransition(new Transition(() -> requestFire, "SHOOT"))
    //         .addTransition(new Transition(() -> requestSort, "SORT"));

    //     states[1] = new State("SHOOT")
    //         .setEntry(() -> {
    //             requestFire = false;
    //             setPower(-1);
    //         })
    //         .setDuring(() -> {
    //             scanDexer();
    //         })
    //         .setFallbackState("IDLE")
    //         .addTransition(new Transition(() -> requestIdle, "IDLE"))
    //         .addTransition(new Transition(() -> requestSort, "SORT"));
        
    //     states[2] = new State("SORT")
    //         .setEntry(() -> {
    //             requestSort = false;
    //         })
    //         .setDuring(() -> {
    //             scanDexer();
    //             if (runPID()) {runSort();}
    //         })
    //         .setFallbackState("IDLE")
    //         .addTransition(new Transition(() -> requestIdle, "IDLE"))
    //         .addTransition(new Transition(() -> requestFire, "SHOOT"));
    
    //     return states;
    // }

    public void update(){
        if (runPID()){
            scanDexer();
            if (sort) {
                int p = 0, g = 0;
                for (String s : stored) {
                    p += s.equals("P") ? 1 : 0;
                    g += s.equals("G") ? 1 : 0;
                }
                if (g + p != 3) {
                    moveEmptySlot();
                } else if (g == 3 || p == 3) { // do nothing
                } else if (g == 1) { // sorting logic
                    target += 120 * ((motif.indexOf("G") - stored.indexOf("G") + 3) % 3);
                } else { // 2g 1p, get 2 correct
                    if (motif.get(stored.indexOf("P")) != "P") {
                        target += 120;
                    }
                }
            } else {
                moveEmptySlot();
            }
        }
    }

    public void shoot(){
        target -= 360;
        for (int i = 2; i >= 0; i--) {
            if (stored.get(i).equals("E")){
                target += 120;
            } else {break;}
        }
    }

    private void moveEmptySlot(){
        target += stored.indexOf("E") % 2 == 0 ? (stored.indexOf("E") + 2) * 120 : 0;
    }

    private void setPower(double pow){
        LServo.setPower(pow);
        RServo.setPower(pow);
    }

    public void scanDexer(){
        stored.clear();
        stored.add(detectColor(colora));
        stored.add(detectColor(colorb));
        stored.add(detectColor(colorc));
    }

    private String detectColor(ColorSensor sensor) {
        status = "YES, THE THING IS RUNNING";
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();
        if (g > r && g > b && g > 2000) {return "G";}
        if (b > g && b > 2000) {return "P";}
        return "E";
    }

    private boolean runPID(){
        setPower(pid.calculate(encoder.getPosition(), target));
        return pid.atSetPoint();
        // return false;
    }

    public boolean isFull(){
        return stored.indexOf("E") == -1;
    }

    public boolean isEmpty(){
        return stored.indexOf("P") == -1 && stored.indexOf("G") == -1;
    }

    public boolean isIdle(){
        return pid.atSetPoint();
    }

    public void enableSort(){sort = true;}
    public void disableSort(){sort = false;}

    public void pleasekillmeiwannadie(){setPower(0.5);}
    public void youbetterflymeouttoworlds(){setPower(-0.5);}
    public void iamsacrificingmyfutureforthis(){setPower(0);}

    @Override
    public String toString(){
        return "Spindexer {" +
                "look here dumdum" + LServo.getPower() + RServo.getPower() +
                "sensorA =" + colora.red() + " " + colora.green() + " " + colora.blue() + stored.get(0) +
                "sensorB =" + colorb.red() + " " + colorb.green() + " " + colorb.blue() + stored.get(1) +
                "sensorC =" + colorc.red() + " " + colorc.green() + " " + colorc.blue() + stored.get(2) +
                status
                + "}";
    }
}
