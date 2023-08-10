package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.NotInUse_OldStuff.Point;

public class Mathematics {
    public static double calculateDerivative(double p0, double p1, double p2, double p3, double t) {
        return 3 * Math.pow(1 - t, 2) * (p1 - p0) + 6 * (1 - t) * t * (p2 - p1) + 3 * Math.pow(t, 2) * (p3 - p2);
    }

    public static double calculateHeading(Point p0, Point p1, Point p2, Point p3, double t) {
        double dx = calculateDerivative(p0.X, p1.X, p2.X, p3.X, t);
        double dy = calculateDerivative(p0.Y, p1.Y, p2.Y, p3.Y, t);
        return Math.atan2(dy, dx);
    }

    public static double encoderTicksToCms(double ticks) {
        return 2.0 * Constants.WHEEL_RADIUS_ODO * Math.PI * ticks / Constants.TICKS_PER_REV;
    }

    public static double AngleWrap(double angle) {
        while (angle < -Math.PI)
            angle += 2 * Math.PI;
        while (angle > Math.PI)
            angle -= 2 * Math.PI;
        return angle;
    }
}
