package org.firstinspires.ftc.teamcode.subsystems;
// push check
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Simple rotating flywheel turret: one motor for the flywheel and one for turret yaw.
 */
public class Turret {
    // private final DcMotorEx flywheel;
    // private final DcMotorEx yawMotor;

    public Turret(HardwareMap hardwareMap) {
        // this.flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        // this.yawMotor = hardwareMap.get(DcMotorEx.class, "turretYaw");

        // flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        // yawMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setFlywheelPower(double power) {
        // flywheel.setPower(power);
    }

    public void setFlywheelVelocity(double ticksPerSecond) {
        // flywheel.setVelocity(ticksPerSecond);
    }

    public void stopFlywheel() {
        // flywheel.setPower(0);
    }

    public void setYawPower(double power) {
        // yawMotor.setPower(power);
    }

    public void brakeYaw() {
        // yawMotor.setPower(0);
    }
}
