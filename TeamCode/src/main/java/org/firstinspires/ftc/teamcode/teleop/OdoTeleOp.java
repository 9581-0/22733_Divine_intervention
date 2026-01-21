package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.Pose2d;

import java.util.Locale;

@TeleOp(name="Odo Teleop", group="Test Teleop")
public class OdoTeleOp extends LinearOpMode{
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode(){
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(0, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        
        waitForStart();

        while(opModeIsActive()){
            odo.update();
            Pose2d pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(),
                pos.getY(),
                pos.getHeading());
            telemetry.addData("Current", data);
            telemetry.update();
        }
    }
}
