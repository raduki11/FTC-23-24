package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

public class BezierCurveGenerating {


    public static List<Pose2d> CubicCurve(Points P0, Points P1, Points P2, Points P3, int NumberOfPoints)
    {
        List<Pose2d> curvePoints = new ArrayList<>();
        double t,X,Y,angle;
        for(double i = 0; i <= NumberOfPoints; i++)
        {
            t = i / NumberOfPoints;
            X = (1-t) * (1-t) * (1-t) * P0.X + 3 * (1-t) * (1-t) * t * P1.X + 3 * (1-t) * t * t * P2.X + t * t * t * P3.X;
            Y = (1-t) * (1-t) * (1-t) * P0.Y + 3 * (1-t) * (1-t) * t * P1.Y + 3 * (1-t) * t * t * P2.Y + t * t * t * P3.Y;
            angle = calculateHeading(P0, P1, P2, P3, t);
            curvePoints.add(new Pose2d(X,Y,angle));
        }
        return (curvePoints);
    }
    public static double calculateDerivative(double p0, double p1, double p2, double p3, double t) {
        return 3 * Math.pow(1-t,2) * (p1 - p0) + 6 * (1-t) * t * (p2 - p1) + 3 * Math.pow(t,2) * (p3 - p2);
    }
    public static double calculateHeading(Points p0, Points p1, Points p2, Points p3, double t) {
        double dx = calculateDerivative(p0.X, p1.X, p2.X, p3.X, t);
        double dy = calculateDerivative(p0.Y, p1.Y, p2.Y, p3.Y, t);
        return Math.atan2(dy, dx);
    }
}
