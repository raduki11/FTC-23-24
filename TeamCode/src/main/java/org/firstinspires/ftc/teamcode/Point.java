package org.firstinspires.ftc.teamcode;

public class Point {
    public double X;
    public double Y;
    public double heading;

    public Point(double x, double y) {
        X = x;
        Y = y;
        this.heading = 0;
    }

    public Point(double x, double y, double heading) {
        X = x;
        Y = y;
        this.heading = heading;
    }
}
