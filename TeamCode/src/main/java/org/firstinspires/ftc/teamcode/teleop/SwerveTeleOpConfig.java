package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SwerveTeleOpConfig {
    // Drive tuning exposed to the dashboard
    public static double Kp = 0.1, Kd = 0.002, Ki = 2, Kf = 0.5, Kl = 0.5;
    public static double module1Adjust = 337, module2Adjust = 285, module3Adjust = 0;
    public static double driveScale = 1.0, rotationScale = 1.0;
    public static boolean useImu = false, fieldCentric = false;

    // Turret tuning
    public static double yawScale = 0.5;
}
