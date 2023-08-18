package org.firstinspires.ftc.teamcode.NotInUse_OldStuff;

import static org.firstinspires.ftc.teamcode.utils.Mathematics.calculateHeading;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class BezierCurveGenerator {

    public static List<Pose2d> CubicCurve(Point P0, Point P1, Point P2, Point P3, int NumberOfPoints) {
        //CubicBezierCurve traj = new CubicBezierCurve(new Vector2D(0, 0), new Vector2D(100, 0), new Vector2D(100, 4), new Vector2D(100, -50));
        List<Pose2d> curvePoints = new ArrayList<>();
        double t, X, Y, angle;
        for (double i = 0; i <= NumberOfPoints; i++) {
            t = i / NumberOfPoints;
            X = (1 - t) * (1 - t) * (1 - t) * P0.X + 3 * (1 - t) * (1 - t) * t * P1.X + 3 * (1 - t) * t * t * P2.X + t * t * t * P3.X;
            Y = (1 - t) * (1 - t) * (1 - t) * P0.Y + 3 * (1 - t) * (1 - t) * t * P1.Y + 3 * (1 - t) * t * t * P2.Y + t * t * t * P3.Y;
            angle = calculateHeading(P0, P1, P2, P3, t);
            //angle = traj.heading(t);
            curvePoints.add(new Pose2d(Y, -X, angle- Math.toRadians(90)));
        }
        return (curvePoints);
    }
}
