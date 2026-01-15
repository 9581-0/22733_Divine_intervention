// package org.firstinspires.ftc.teamcode.util.templates;

// //import com.acmerobotics.dashboard.FtcDashboard;
// //import com.acmerobotics.dashboard.config.Config;
// //import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// import org.firstinspires.ftc.teamcode.util.PIDF;
// import org.firstinspires.ftc.teamcode.util.wrappers.Hardware;
// import org.firstinspires.ftc.teamcode.util.wrappers.PosEncoder;
// import org.firstinspires.ftc.teamcode.util.wrappers.nCRServo;
// import org.firstinspires.ftc.teamcode.util.wrappers.nMotor;

// //@Config
// @Disabled
// public abstract class PIDF_Tuner extends OpMode {

//     public static double P = 0;
//     public static double D = 0;
//     public static double F = 0;
//     public static double targetPos = 0;
//     public static double tolerance = 0;

// //    MultipleTelemetry telemetryData = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//     private static final PIDF controller = new PIDF(P, D, F);
//     private Hardware hardware;

//     @Override
//     public void init() {
//         hardware = setHardware();
//     }

//     @Override
//     public void loop() {
//         controller.setP(P);
//         controller.setD(D);
//         controller.setF(F);
//         controller.setTolerance(tolerance);
//         double power = 0;

// //        telemetryData.addData("power", power);
// //        telemetryData.addData("target pos", targetPos);

//         if (hardware.getClass() == nMotor.class) {
//             nMotor motor = (nMotor) hardware;
//             double currentPos = motor.getPosition();

//             power = controller.calculate(currentPos, targetPos);

//             motor.setPower(power);

// //            telemetryData.addData("actual pos", currentPos);
//         } else if (hardware.getClass() == nCRServo.class) {
//             nCRServo servo = (nCRServo) hardware;
//             double currentPos = ((PosEncoder)servo.getEncoder()).getPosition();

//             power = controller.calculate(currentPos, targetPos);
            
//             servo.setPower(power);

// //            telemetryData.addData("actual pos", currentPos);
//         }

// //        telemetryData.update();
//     }

//     public abstract Hardware setHardware();
// }