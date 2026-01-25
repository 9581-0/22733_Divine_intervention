package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

public class CrServoCaching implements CRServo {
        protected final CRServo crServo;
        protected double cachingTolerance;
        private double cachedPower = Double.NaN;

        /*
         * @param crServo the continuous rotation servo to encapsulate in the caching control
         * @param cachingTolerance the power delta threshold at which a power write will occur.
         */
        public CrServoCaching(CRServo crServo, CRServo crServo1) {
            this.crServo = crServo1;
            new CrServoCaching(crServo, 0.005);
        }

        public CrServoCaching(CRServo crServo, double cachingTolerance) {
            this.crServo = crServo;
            this.cachingTolerance = cachingTolerance;
        }

        @Override
        public void setDirection(Direction direction) {

        }

        @Override
        public Direction getDirection() {
            return null;
        }

        /**
         * Checks if the change in [power] since last write exceeds [cachingTolerance], if so, does a hardware write (actually sets the power)
         *
         * @see com.qualcomm.robotcore.hardware.CRServo#setPower(double)
         * @param power the new power level of the servo, a value in the interval [-1.0, 1.0]
         */
        @Override
        public void setPower(double power) {
            double corrected = Math.max(-1.0, Math.min(1.0, power));
            synchronized (this) {
                if (Math.abs(corrected - cachedPower) >= cachingTolerance
                        || (corrected == 0.0 && cachedPower != 0.0)
                        || (corrected >= 1.0 && !(cachedPower >= 1.0))
                        || (corrected <= -1.0 && !(cachedPower <= -1.0))
                        || Double.isNaN(cachedPower)) {
                    cachedPower = corrected;
                    crServo.setPower(corrected);
                }
            }
        }

        /**
         * Checks if the change in [power] since last write exceeds [cachingTolerance], if so, does a hardware write (actually sets the power)
         * @see #setPower(double)
         * @see com.qualcomm.robotcore.hardware.CRServo#setPower(double)
         * @param power the new power level of the servo, a value in the interval [-1.0, 1.0]
         * @return if a hardware write to update the output to the servo was executed
         */
        public boolean setPowerResult(double power) {
            double corrected = Math.max(-1.0, Math.min(1.0, power));
            synchronized (this) {
                if (Math.abs(corrected - cachedPower) >= cachingTolerance
                        || (corrected == 0.0 && cachedPower != 0.0)
                        || (corrected >= 1.0 && !(cachedPower >= 1.0))
                        || (corrected <= -1.0 && !(cachedPower <= -1.0))
                        || Double.isNaN(cachedPower)) {
                    cachedPower = corrected;
                    crServo.setPower(corrected);
                    return true;
                }
            }
            return false;
        }

        /**
         * Sets [cachingTolerance] to 0 temporarily, then performs and returns [setPowerResult]
         */
        public boolean setPowerRaw(double power) {
            synchronized (this) {
                double originalTolerance = this.cachingTolerance;
                this.cachingTolerance = 0.0;
                boolean res = setPowerResult(power);
                this.cachingTolerance = originalTolerance;
                return res;
            }
        }

        // Delegate other CRServo methods to crServo

        @Override
        public void close() {
            crServo.close();
        }

        @Override
        public Manufacturer getManufacturer() {
            return null;
        }

        @Override
        public String getDeviceName() {
            return crServo.getDeviceName();
        }

        @Override
        public String getConnectionInfo() {
            return crServo.getConnectionInfo();
        }

        @Override
        public int getVersion() {
            return crServo.getVersion();
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {
            crServo.resetDeviceConfigurationForOpMode();
        }

        @Override
        public double getPower() {
            return crServo.getPower();
        }

        @Override
        public ServoController getController() {
            return null;
        }

        @Override
        public int getPortNumber() {
            return 0;
        }
}