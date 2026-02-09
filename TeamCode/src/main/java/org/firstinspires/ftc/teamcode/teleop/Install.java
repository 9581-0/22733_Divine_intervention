package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp
public class Install extends OpMode {
    Turret turret;
    
    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        turret.setHalf();
    }

    @Override
    public void loop() {
        turret.setHalf();
    }

}
