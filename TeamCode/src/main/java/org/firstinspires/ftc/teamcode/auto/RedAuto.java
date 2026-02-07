package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Red Auto", group = "Auton", preselectTeleOp = "TeleOP")
public class RedAuto extends OpMode {
    @Override
    public void init() {
        Turret.GOAL_X = 144.0;
        Turret.GOAL_Y = 144.0;
    }

    @Override
    public void loop() {

    }
}
