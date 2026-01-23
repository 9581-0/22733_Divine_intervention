package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Pose2d implements Cloneable {
    public double x;
    public double y;
    public double heading;

    public Pose2d(double x, double y){
        this(x,y,0);
    }

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public void add(Pose2d p1){
        this.x += p1.x;
        this.y += p1.y;
        this.heading += p1.heading;
    }

    public boolean isNaN(){
        return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading);
    }

    public double getX(){ return x; }
    public double getY(){
        return y;
    }
    public double getHeading(){
        return heading;
    }

    public double getDistanceFromPoint(Pose2d newPoint) { // distance equation
        return Math.sqrt(Math.pow((x - newPoint.x),2) + Math.pow((y - newPoint.y),2));
    }

    public double getErrorInX(Pose2d newPoint) { // distance equation
        return Math.abs(x - newPoint.x);
    }

    public double getErrorInY(Pose2d newPoint) { // distance equation
        return Math.abs(y - newPoint.y);
    }

    public void clipAngle() {
        heading = clipAngle(heading);
    }

    public static double clipAngle(double angle) {
        while (Math.abs(angle) > Math.PI) {
            angle -= Math.PI * 2.0 * Math.signum(angle);
        }
        return angle;
    }

    // public Pose2d convertPose2D(Pose2D pose){
    //     return new Pose2d(pose.getX(), pose.getY(), pose.getHeading());
    // }

    @NonNull
    @Override
    public Pose2d clone() {
        return new Pose2d(x, y, heading);
    }

    @SuppressLint("DefaultLocale")
    public String toString() {
        return String.format("(%.3f, %.3f, %.3f", x, y, heading);
    }
}