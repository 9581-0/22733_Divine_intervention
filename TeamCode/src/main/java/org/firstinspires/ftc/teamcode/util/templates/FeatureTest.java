// package org.firstinspires.ftc.teamcode.util.templates;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// import org.firstinspires.ftc.teamcode.util.wrappers.Hardware;
// import org.firstinspires.ftc.teamcode.util.wrappers.nServo;

// import java.util.ArrayList;

// public class FeatureTest extends LinearOpMode {
//     private Feature feature;


//     @Override
//     public void runOpMode() throws InterruptedException {
//         ArrayList<Hardware> hw = feature.hw;
//         init(hw);
//         int index = 0;

//         waitForStart();

//         while (opModeIsActive()) {
//             if (gamepad1.leftBumperWasPressed()) {
//                 if (index == 0) {
//                     index = hw.size() - 1;
//                 } else {
//                     index--;
//                 }
//             } else if (gamepad1.rightBumperWasPressed()) {
//                 if (index == hw.size() - 1) {
//                     index = 0;
//                 } else {
//                     index++;
//                 }
//             }

//             Hardware cur = hw.get(index);
//             if (cur instanceof nServo) {
//                 servoControl((nServo) cur);
//             }

//             telemetry.addData("Current Hardware: ", cur.getName());
//             for (Hardware item : hw) {
//                 if (item instanceof nServo) {
//                     nServo servo = (nServo) item;
//                     telemetry.addData(String.format("%s position: ", servo.name), servo.getCurrentPosition());
//                     telemetry.addData(String.format("%s angle: ", servo.name), servo.getCurrentAngle());
//                 }
//             }

//             telemetry.update();
//         }
//     }

//     protected FeatureTest setFeature (Feature feature) {
//         this.feature = feature;
//         return this;
//     }

//     private void init(ArrayList<Hardware> hw) {
//         for (Hardware item : hw) {
//             if (item instanceof nServo) {
//                 ((nServo) item).setTargetPos(0);
//             }
//         }
//     }
    
//     private void servoControl(nServo servo) {
//         double curPos = servo.getCurrentPosition();

//         if (gamepad1.dpadUpWasPressed()) {
//             servo.setTargetPos(Math.min(curPos + 0.1, 1));
//         } else if (gamepad1.dpadDownWasPressed()) {
//             servo.setTargetPos(Math.max(curPos - 0.1, 0));
//         } else if (gamepad1.dpadRightWasPressed()) {
//             servo.setTargetPos(Math.min(curPos + 0.05, 1));
//         } else if (gamepad1.dpadLeftWasPressed()) {
//             servo.setTargetPos(Math.max(curPos - 0.05, 0));
//         }

//         double curAngle = servo.getCurrentAngle();

//         if (gamepad1.yWasPressed()) {
//             servo.setTargetAngle(Math.min(curAngle + Math.toRadians(10), servo.getMaxRadians()));
//         } else if (gamepad1.aWasPressed()) {
//             servo.setTargetAngle(Math.max(curAngle - Math.toRadians(10), 0));
//         } else if (gamepad1.bWasPressed()) {
//             servo.setTargetAngle(Math.min(curAngle + Math.toRadians(5), servo.getMaxRadians()));
//         } else if (gamepad1.xWasPressed()) {
//             servo.setTargetAngle(Math.max(curAngle - Math.toRadians(5), 0));
//         }

//         telemetry.addLine("Y/A +- 10 radians\nB/X +- 5 radians");
//         telemetry.addLine("DPAD ▲▼ +- 0.1 position\nDPAD ◀▶ +- 0.05 position");
//     }
// }