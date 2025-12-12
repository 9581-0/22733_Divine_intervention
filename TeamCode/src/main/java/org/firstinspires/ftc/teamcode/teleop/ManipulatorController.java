package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class ManipulatorController {
    public enum IndexerState { LEFT, RIGHT, HOLD }

    public static class ManipulatorState {
        private final double yawPower;
        private final double flywheelPower;
        private final IndexerState indexerState;

        public ManipulatorState(double yawPower, double flywheelPower, IndexerState indexerState) {
            this.yawPower = yawPower;
            this.flywheelPower = flywheelPower;
            this.indexerState = indexerState;
        }

        public double getYawPower() {
            return yawPower;
        }

        public double getFlywheelPower() {
            return flywheelPower;
        }

        public IndexerState getIndexerState() {
            return indexerState;
        }
    }

    // private final Turret turret;
    // private final Indexer indexer;

    public ManipulatorController(Turret turret, Indexer indexer) {
        // this.turret = turret;
        // this.indexer = indexer;
    }

    public ManipulatorState updateFromGamepad(Gamepad gamepad) {
        // double yawInput = gamepad.right_stick_x * SwerveTeleOpConfig.yawScale;
        // turret.setYawPower(yawInput);

        // double flywheelPower = gamepad.left_trigger - gamepad.right_trigger;
        // if (Math.abs(flywheelPower) > 0.01) {
        //     turret.setFlywheelPower(flywheelPower);
        // } else {
        //     turret.stopFlywheel();
        // }

        IndexerState indexerState = IndexerState.HOLD;
        // if (gamepad.dpad_left) {
        //     indexer.left();
        //     indexerState = IndexerState.LEFT;
        // } else if (gamepad.dpad_right) {
        //     indexer.right();
        //     indexerState = IndexerState.RIGHT;
        // } else if (gamepad.dpad_down) {
        //     indexer.neutral();
        // }

        return new ManipulatorState(0, 0, indexerState);
    }
}
