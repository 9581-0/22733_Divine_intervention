package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SwerveTeleOpConfig {

    // =========================================================
    // 1. PID COEFFICIENTS (Edit these live to tune holding power)
    // =========================================================
    public static double Kp = 0.05;
    public static double Kd = 0.0;
    public static double Ki = 0; // Usually 0 for drive modules
    public static double Kf = 0;
    public static double Kl = 0.0; // Integral limit

    // =========================================================
    // 2. MODULE OFFSETS (Values from your Zeroing OpMode)
    // =========================================================
    // Enter the raw angles you read when wheels are pointing FORWARD
    public static double module1Adjust = 60;
    public static double module2Adjust = -90;
    public static double module3Adjust = -140;

    // =========================================================
    // 3. DRIVER PREFERENCES
    // =========================================================
    public static double DRIVE_SPEED_SCALAR = 1.0;
    public static double ROTATION_SPEED_SCALAR = 1.0;
    public static boolean FIELD_CENTRIC = true;
    public static boolean USE_IMU = true;
    
    // Set to -1 if rotating the robot clockwise makes heading DECREASE
    public static double IMU_POLARITY = -1.0; 

    // =========================================================
    // 4. KINEMATICS / GEOMETRY
    // =========================================================
    // Effective radius of rotation. Increase this if the robot spins too fast/slow relative to drive.
    // In your old code this was implicitly "1.0".
    public static double ROBOT_RADIUS = 1.0; 
}