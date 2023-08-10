package z_NotInUse_OldStuff;

import static utils.Mathematics.calculateHeading;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class BezierCurveGenerator {


    public static List<Pose2d> CubicCurve(Point P0, Point P1, Point P2, Point P3, int NumberOfPoints) {
        List<Pose2d> curvePoints = new ArrayList<>();
        double t, X, Y, angle;
        for (double i = 0; i <= NumberOfPoints; i++) {
            t = i / NumberOfPoints;
            X = (1 - t) * (1 - t) * (1 - t) * P0.X + 3 * (1 - t) * (1 - t) * t * P1.X + 3 * (1 - t) * t * t * P2.X + t * t * t * P3.X;
            Y = (1 - t) * (1 - t) * (1 - t) * P0.Y + 3 * (1 - t) * (1 - t) * t * P1.Y + 3 * (1 - t) * t * t * P2.Y + t * t * t * P3.Y;
            angle = calculateHeading(P0, P1, P2, P3, t);
            curvePoints.add(new Pose2d(Y, -X, angle - Math.toRadians(90)));
        }
        return (curvePoints);
    }
}
