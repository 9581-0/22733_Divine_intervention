package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class ServoCaching {
    public class CachingServo implements Servo {
        protected final Servo servo;
        protected double cachingTolerance = 0.001;
        private double cachedPosition = Double.NaN;

        /*
         * @param servo the servo to encapsulate in the caching control
         * @param cachingTolerance the position delta threshold at which a position write will occur.
         */
        public CachingServo(Servo servo) {
            this(servo, 0.001);
        }

        public CachingServo(Servo servo, double cachingTolerance) {
            this.servo = servo;
            this.cachingTolerance = cachingTolerance;
        }

        /*
         * Checks if the change in [position] since last write exceeds [cachingTolerance], if so, does a hardware write (actually sets the position)
         *
         * @param position the position to which the servo should move, a value in the range [0.0, 1.0]
         * @see com.qualcomm.robotcore.hardware.Servo#setPosition(double)
         * @see ServoController#pwmEnable()
         */
        @Override
        public void setPosition(double position) {
            double corrected = Math.min(1.0, Math.max(0.0, position));
            synchronized (this) {
                if (Math.abs(corrected - cachedPosition) >= cachingTolerance
                        || (corrected <= 0.0 && !(cachedPosition <= 0.0))
                        || (corrected >= 1.0 && !(cachedPosition >= 1.0))
                        || Double.isNaN(cachedPosition)) {
                    cachedPosition = corrected;
                    servo.setPosition(corrected);
                }
            }
        }

        /**
         * Checks if the change in [position] since last write exceeds [cachingTolerance], if so, does a hardware write (actually sets the position)
         *
         * @param position the position to which the servo should move, a value in the range [0.0, 1.0]
         * @return if a hardware write to update the output to the servo was executed
         * @see #setPosition(double)
         * @see com.qualcomm.robotcore.hardware.Servo#setPosition(double)
         */
        public boolean setPositionResult(double position) {
            double corrected = Math.min(1.0, Math.max(0.0, position));
            synchronized (this) {
                if (Math.abs(corrected - cachedPosition) >= cachingTolerance
                        || (corrected <= 0.0 && !(cachedPosition <= 0.0))
                        || (corrected >= 1.0 && !(cachedPosition >= 1.0))
                        || Double.isNaN(cachedPosition)) {
                    cachedPosition = corrected;
                    servo.setPosition(corrected);
                    return true;
                }
            }
            return false;
        }

        /**
         * Sets [cachingTolerance] to 0 temporarily, then performs and returns [setPositionResult]
         */
        public boolean setPositionRaw(double position) {
            synchronized (this) {
                double originalCachingTolerance = this.cachingTolerance;
                this.cachingTolerance = 0.0;
                boolean res = setPositionResult(position);
                this.cachingTolerance = originalCachingTolerance;
                return res;
            }
        }

        // Delegate other Servo interface methods to the encapsulated servo

        @Override
        public double getPosition() {
            return servo.getPosition();
        }

        @Override
        public void scaleRange(double min, double max) {
            servo.scaleRange(min, max);
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {
            servo.resetDeviceConfigurationForOpMode();
        }

        @Override
        public void close() {
            servo.close();
        }

        @Override
        public Manufacturer getManufacturer() {
            return null;
        }

        @Override
        public String getDeviceName() {
            return servo.getDeviceName();
        }

        @Override
        public String getConnectionInfo() {
            return servo.getConnectionInfo();
        }

        @Override
        public int getVersion() {
            return servo.getVersion();
        }

        @Override
        public ServoController getController() {
            return servo.getController();
        }

        @Override
        public int getPortNumber() {
            return servo.getPortNumber();
        }

        @Override
        public void setDirection(Direction direction) {

        }

        @Override
        public Direction getDirection() {
            return null;
        }
    }
}