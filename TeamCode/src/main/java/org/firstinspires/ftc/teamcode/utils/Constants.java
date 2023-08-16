package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static final double Horizontal_Offset = 21.2,  //cm
            Forward_Offset = -5.5;  //cm

    public static final double WHEEL_RADIUS_ODO = 1.75; //cm roata odo
    public static final double TICKS_PER_REV = 8192;
    public static final double X_Multiplier = 1.0155,
            Y_Multiplier = 1.028;
    public static double kP_X = 0.01, kD_X = 0.005,
            kP_Y = 0.01, kD_Y = 0.005,
            kP_Headng = 0.5, kD_Heading = 0.007;
}
