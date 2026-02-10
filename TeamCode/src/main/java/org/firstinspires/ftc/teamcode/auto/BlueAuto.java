package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Blue Auto", group = "Auton", preselectTeleOp = "TeleOP")
public class BlueAuto extends OpMode {
    @Override
    public void init() {
        Turret.GOAL_X = -72.0;
        Turret.GOAL_Y = 72.0;
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
    }

}
