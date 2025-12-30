package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.utility.myDcMotorEx;

public class Shooter {

    private final myDcMotorEx ShooterMotor;

    private final PIDcontroller ShooterPID = new PIDcontroller(0, 0, 0, 0, 0);

    final double ticks_per_rev = 28;
    double target_rpm = 6000;
    double penetration = 0;
    double angle = 0;


    public Shooter(HardwareMap hardwareMap) {
        ShooterMotor = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "SM"));
        ShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shoot(boolean shooting) {
        if (shooting) {
            ShooterMotor.setVelocity(target_rpm / ticks_per_rev);
        } else ShooterMotor.setPower(0);
}

}