package utils;

public class Constants {
    public static final double  Horizontal_Offset = 21.2,  //cm
                                Forward_Offset = -5.5;  //cm

    public static final double    WHEEL_RADIUS_ODO = 1.75; //cm roata odo
    public static final double    TICKS_PER_REV = 8192;
    public static final double    BASE = 0.02;
    public static final double WHEEL_RADIUS = 0.048;//cm Mecanum
    public static final double      X_Multiplier = 1,
                                    Y_Multiplier = 1;
    public static double kP_posX = 0.002, kI_posX = 0, kD_posX = 0.0004,
            kP_posY = 0.002, kI_posY = 0, kD_posY = 0.0004,
            kP_heading = 0.1, kI_heading = 0, kD_heading = 0,
            kF = 0;
}
