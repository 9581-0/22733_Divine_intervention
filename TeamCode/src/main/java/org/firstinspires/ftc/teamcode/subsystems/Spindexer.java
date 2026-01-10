package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Spindexer {

    private final Servo IndexServoL;

    private final Servo IndexServoR;

     public Spindexer(HardwareMap hardwareMap){
         IndexServoL = hardwareMap.get(Servo.class, "IndexServoL");
         IndexServoR = hardwareMap.get(Servo.class, "IndexServoR");
     }

    public void homing(){}

    public void Spin2win(){}




}
