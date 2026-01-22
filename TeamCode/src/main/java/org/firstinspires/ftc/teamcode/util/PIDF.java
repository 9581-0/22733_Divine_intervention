package org.firstinspires.ftc.teamcode.util;

import java.util.function.Supplier;

public class PIDF {

    private Supplier<Double> kP, kD, kF;
    private double setPoint;
    private double reportedValue;

    private double pError, vError;
    private double totalError, prevErrorVal;
    private double pTolerance = 0.05, vTolerance = Double.POSITIVE_INFINITY;

    private double lastTimeStamp, period;

    public PIDF (double kp, double kd) {
        this(kp, kd, 0);
    }
    public PIDF (Supplier<Double> kp, Supplier<Double> kd) {
        this(kp, kd, () -> 0.0, 0, 0);
    }

    public PIDF (double kp, double kd, double kf) {
        this(() -> kp, () -> kd, () -> kf, 0, 0);
    }

    public PIDF (Supplier<Double> kp, Supplier<Double> kd, Supplier<Double> kv) {
        this(kp, kd, kv, 0, 0);
    }

    public PIDF (double kp, double kd, Supplier<Double> kV) {
        this(() -> kp, () -> kd, kV, 0, 0);
    }

    public PIDF (Supplier<Double> kP, Supplier<Double> kD, Supplier<Double> kF, double sp, double pv) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;

        setPoint = sp;
        reportedValue = pv;

        lastTimeStamp = 0;
        period = 0;

        pError = setPoint - reportedValue;
        reset();
    }

    public void reset() {
        totalError = 0;
        prevErrorVal = 0;
        lastTimeStamp = 0;
    }

    public void setTolerance(double pTolerance) {
        setTolerance(pTolerance, Double.POSITIVE_INFINITY);
    }

    public void setTolerance(double pTolerance, double vTolerance) {
        this.pTolerance = pTolerance;
        this.vTolerance = vTolerance;
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void setSetPoint(double sp) {
        setPoint = sp;
        pError = setPoint - reportedValue;
        vError = (pError - prevErrorVal) / period;
    }

    public boolean atSetPoint (double pTolerance) {
        setTolerance(pTolerance);
        return atSetPoint();
    }

    public boolean atSetPoint () {
        return Math.abs(pError) < pTolerance && Math.abs(vError) < vTolerance;
    }

    public double[] getCoeffs() {
        return new double[]{kP.get(), kD.get(), kF.get()};
    }

    public double[] getTolerance() {
        return new double[]{pTolerance, vTolerance};
    }

    public double getPositionalError() {
        return pError;
    }

    public double getVelocityError() {
        return vError;
    }

    public double calculate () {
        return calculate(reportedValue);
    }

    public double calculate (double pv, double sp) {
        setSetPoint(sp);
        return calculate(pv);
    }

    public double calculate (double pv) {
        prevErrorVal = pError;

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        pError = setPoint - pv;
        reportedValue = pv;

        vError = period > 1e-6 ? (pError - prevErrorVal) / period : 0.0;

        totalError = period * (setPoint - reportedValue);

        return (kP.get() * pError) + kD.get() * vError + (Math.signum(pError) * kF.get());
    }

    public void setPDF (double kP, double kF, double kD) {
        setP(kP);
        setF(kF);
        setD(kD);
    }
    public void clearTotalError() {
        totalError = 0;
    }

    public void setP(double kp) {
        kP = () -> kp;
    }

    public void setD(double kd) {
        kD = () -> kd;
    }

    public void setF(double kf) {
        kF = () -> kf;
    }

    public double getP() {
        return kP.get();
    }

    public double getD() {
        return kD.get();
    }

    public double getF() {
        return kF.get();
    }

    public double getPeriod() {
        return period;
    }
}