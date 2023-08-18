package org.firstinspires.ftc.teamcode.Algorithms;

import static org.firstinspires.ftc.teamcode.utils.Constants.CORRECTION_DISTANCE;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.utils.Vector2D;

public class GVFPathFollowing {
    public Pose2d calcGuidanceVector(CubicBezierCurve curve, Vector2D curr_pos, double heading) {
        double closestT = findClosestPoint(curve, curr_pos);
        Vector2D closestPoint = curve.calculate(closestT);
        Vector2D robotToClosestPoint = closestPoint.subtract(curr_pos);
        Vector2D endPoint = curve.calculate(1);
        /**
         double directPursuitThreshold = 1;
         for (double i = 1; i >= 0; i -= 1 / 10.0) {
         double dist = endPoint.subtract(curve.calculate(i)).getMagSq();
         if (dist <= SAVING_THROW_DISTANCE) {
         directPursuitThreshold = i;
         break;
         }
         }
         */
        Vector2D robotToEnd = endPoint.subtract(curr_pos);
        double correctionFactor = Math.min(1, robotToClosestPoint.getMagnitude() / CORRECTION_DISTANCE);
        double movementDirection = hlerp(curve.heading(closestT), robotToClosestPoint.getHeading(), correctionFactor);

        if (closestT == 1) //&& Math.abs(curve.heading(closestT) - curr_pos.subtract(closestPoint).getHeading()) <= 0.5 * Math.PI))
            movementDirection = endPoint.subtract(curr_pos).getHeading();

        Vector2D movementVector = new Vector2D(Math.cos(movementDirection), Math.sin(movementDirection));
        double speed = 1;
        if (robotToEnd.getMagnitude() <= 200) {
            speed = lerp(0.5, speed, robotToEnd.getMagnitude() / 200);
        }

        movementVector = movementVector.scalarMultiply(speed);
        return new Pose2d(movementVector.getX(), movementVector.getY(), (curve.heading(closestT) - heading) * speed * 2);
    }


    public double hlerp(double a, double b, double t) {
        double diff = b - a;
        diff %= 2 * Math.PI;
        if (Math.abs(diff) > Math.PI) {
            if (diff > 0) {
                diff -= 2 * Math.PI;
            } else {
                diff += 2 * Math.PI;
            }
        }
        return a + t * diff;
    }

    private double lerp(double a, double b, double t) {
        return (1 - t) * a + t * b;
    }

    public double findClosestPoint(CubicBezierCurve curve, Vector2D point) {
        //long startTime = System.nanoTime();
        double minT = -1;
        double minDist = Double.POSITIVE_INFINITY;
        int numOfPoints = 200;
        for (int i = 0; i <= numOfPoints; i++) {
            double t = i / (double) numOfPoints;
            double dist = calculateMinimizationFunction(curve, t, point);
            if (dist < minDist) {
                minDist = dist;
                minT = t;
            }
        }
        return minT;
    }

    private double calculateMinimizationFunction(CubicBezierCurve curve, double t, Vector2D point) {
        return curve.calculate(t).subtract(point).getMagSq();
    }
}
