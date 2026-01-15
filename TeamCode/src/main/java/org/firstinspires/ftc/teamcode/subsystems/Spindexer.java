package org.firstinspires.ftc.teamcode.subsystems;

import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.drivers.REVColorSensorV3;
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.wrappers.Sensorange;
import org.firstinspires.ftc.teamcode.util.statemachine.State;
import org.firstinspires.ftc.teamcode.util.statemachine.StateMachine;

public class Spindexer {
    private CRServo LServo;
    private CRServo RServo;

    private REVColorSensorV3 colora;
    private REVColorSensorV3 colorb;
    private REVColorSensorV3 colorc;
    private Sensorange encoder;

    private StateMachine state;
    public static double P = 0.05, D = 0.001;
    private PIDF pid = new PIDF(P, D);
    private double target;

    private ArrayList<String> motif = new ArrayList<>();
    private ArrayList<String> stored = new ArrayList<>();
    private boolean requestFire, requestSort, requestIdle = false;
    private boolean sort = false;


    public Spindexer(HardwareMap map){
        LServo = map.get(CRServo.class, "IndexServoL");
        RServo = map.get(CRServo.class, "IndexServoR");

        encoder = new Sensorange("encoder", map);

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
            stored = scanDexer();
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

    public ArrayList scanDexer(){
        stored.clear();
        stored.add(detectColor(colora));
        stored.add(detectColor(colorb));
        stored.add(detectColor(colorc));
        return stored;
    }

    private String detectColor(REVColorSensorV3 sensor) {
        float[] color = sensor.readLSRGB();

        if (color[1] > color[0] && color[1] > color[0] && color[1] > 100) {return "G";}
        if (color[2] > 70 && color[0] > 70 && color[1] < 70) {return "P";}
        return "E";
    }

    private boolean runPID(){
        setPower(pid.calculate(encoder.getPosition(), target));
        return pid.atSetPoint();
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

    @Override
    public String toString(){
        float[] ca = colora.readLSRGB();
        float[] cb = colorb.readLSRGB();
        float[] cc = colorc.readLSRGB();
        return "Spindexer {" +
                "sensorA =" + ca[0] + " " + ca[1] + " " + ca[2] + stored.get(0) +
                "sensorB =" + cb[0] + " " + cb[1] + " " + cb[2] + stored.get(1) +
                "sensorC =" + cc[0] + " " + cc[1] + " " + cc[2] + stored.get(2)
                + "}";
    }
}
