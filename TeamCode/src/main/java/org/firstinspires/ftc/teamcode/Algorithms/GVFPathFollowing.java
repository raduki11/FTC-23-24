package org.firstinspires.ftc.teamcode.Algorithms;

import org.firstinspires.ftc.teamcode.utils.Vector2D;

public class GVFPathFollowing {
    public Vector2D calcGuidanceVector(CubicBezierCurve curve, Vector2D curr_pos) {
        long startTime = System.nanoTime();
        double closestT = findClosestPoint(curve, curr_pos);
        Vector2D closestPoint = curve.calculate(closestT);
        Vector2D curveDerivative = curve.derivative(closestT);
        Vector2D robotToClosestPoint = closestPoint.subtract(curr_pos);
        Vector2D endPoint = curve.calculate(1);
        double CORRECTION_DISTANCE = 100;
        double SAVING_THROW_DISTANCE = 100;
        double directPursuitThreshold = 1;
        for (double i = 1; i >= 0; i -= 1 / 100.0) {
            double dist = endPoint.subtract(curve.calculate(i)).getMagSq();
            if (dist > SAVING_THROW_DISTANCE) {
                directPursuitThreshold = i;
                break;
            }
        }
        Vector2D robotToEnd = endPoint.subtract(curr_pos);
        double correctionFactor = Math.min(1, robotToClosestPoint.getMagnitude() / CORRECTION_DISTANCE);
        double movementDirection = hlerp(curveDerivative.getHeading(), robotToClosestPoint.getHeading(), correctionFactor);
        if ((closestT == 1 && Math.abs(curr_pos.subtract(closestPoint).getHeading() - curveDerivative.getHeading()) <= 0.5 * Math.PI) ||
                closestT >= directPursuitThreshold) {
            movementDirection = endPoint.subtract(curr_pos).getHeading();
        }

        Vector2D movementVector = new Vector2D(Math.cos(movementDirection), Math.sin(movementDirection));
        double speed = 1;
        if (robotToEnd.getMagnitude() < 200) {
            speed = lerp(0.2, speed, robotToEnd.getMagnitude() / 200);
        }

        movementVector = movementVector.scalarMultiply(speed);
        return movementVector;
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
        int numOfPoints = 100;
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
