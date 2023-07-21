package utils;

public class Constants {
    public static final double Horizontal_Offset = 21.2,  //cm
            Forward_Offset = -5.5;  //cm

    public static final double    WHEEL_DIAMETER = 3.5; //cm roata odo
    public static final double    TICKS_PER_REV = 8192;
    public static final double    BASE = 21;
    public static final double WHEEL_RADIUS = 9.6;//cm Mecanum
    public static final double      X_Multiplier = 1.01416,
                                    Y_Multiplier = 1.0686;
    public static double kP_posX = 0.4, kI_posX = 0, kD_posX = 0.08,
            kP_posY = 0.4, kI_posY = 0, kD_posY = 0.08,
            kP_heading = 0.4, kI_heading = 0, kD_heading = 0;  //ToDo coeficienti PID
}
