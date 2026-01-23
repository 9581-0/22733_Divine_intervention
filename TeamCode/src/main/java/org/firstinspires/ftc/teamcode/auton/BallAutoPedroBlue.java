package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.vision.LimelightBallTracker.BallColor;

@Autonomous(name = "Ball Auto (Pedro, Blue)", group = "Auton")
public class BallAutoPedroBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        BallAutoPedroHelper.runAuto(this, BallColor.BLUE);
    }
}
