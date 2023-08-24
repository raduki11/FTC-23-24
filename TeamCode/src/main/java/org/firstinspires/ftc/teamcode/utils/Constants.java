package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    //Localization Constants ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    public static final double Horizontal_Offset = 21.6;  //cm
    public static final double Forward_Offset = 4.8;  //cm


    public static final double WHEEL_RADIUS_ODO = 1.75; //cm roata odo
    public static final double TICKS_PER_REV = 8192;

    public static final double X_Multiplier = 1.0155;
    public static final double Y_Multiplier = 1.028;
    //Localization Constants ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


    //GoToPoint Constants ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    public static double kP_X = 0.01, kD_X = 0.005,
            kP_Y = 0.01, kD_Y = 0.005,
            kP_Headng = 0.8, kD_Heading = 0.007;
    //GoToPoint Constants ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


    //GVF Constants ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    public static double CORRECTION_DISTANCE = 5;
    public static double SAVING_THROW_DISTANCE = 30;
    //GVF Constants ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
}
