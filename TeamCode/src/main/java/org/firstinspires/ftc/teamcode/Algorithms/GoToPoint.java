package org.firstinspires.ftc.teamcode.Algorithms;

import static org.firstinspires.ftc.teamcode.utils.Constants.kD_Heading;
import static org.firstinspires.ftc.teamcode.utils.Constants.kD_X;
import static org.firstinspires.ftc.teamcode.utils.Constants.kD_Y;
import static org.firstinspires.ftc.teamcode.utils.Constants.kP_Headng;
import static org.firstinspires.ftc.teamcode.utils.Constants.kP_X;
import static org.firstinspires.ftc.teamcode.utils.Constants.kP_Y;
import static org.firstinspires.ftc.teamcode.utils.Mathematics.AngleWrap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class GoToPoint {
    private double prev_error_heading = 0, prev_time = 0, prev_error_X = 0, prev_error_Y = 0;

    public Pose2d goToPoint(ElapsedTime timer, Pose2d current_point, Pose2d target_point, double movementSpeed, double turnSpeed) {
        Pose2d movement = new Pose2d(0, 0, 0);
        Pose2d err = target_point.minus(current_point);
        double dist = Math.hypot(err.getX(), err.getY());
        if (dist <= 2 && Math.toDegrees(Math.abs(err.getHeading())) <= 1) {
            movement = new Pose2d(0, 0, 0);
        } else {
            double P_X = kP_X * err.getX();
            double P_Y = kP_Y * err.getY();
            double P_heading = kP_Headng * err.getHeading();

            double delta_time = timer.seconds() - prev_time;
            double D_X = kD_X * (err.getX() - prev_error_X) / delta_time;
            double D_Y = kD_Y * (err.getY() - prev_error_Y) / delta_time;
            double D_heading = kD_Heading * (err.getHeading() - prev_error_heading) / delta_time;

            prev_time = timer.seconds();
            prev_error_X = err.getX();
            prev_error_Y = err.getY();
            prev_error_heading = err.getHeading();

            double absoluteAngleToTarget = Math.atan2(err.getX(), err.getY());
            double relativeAngleToTarget = AngleWrap(absoluteAngleToTarget - current_point.getHeading());

            double relX = Math.sin(relativeAngleToTarget) * dist;
            double relY = Math.cos(relativeAngleToTarget) * dist;
            //double rel_Turn = target_point.getHeading() - relativeAngleToTarget;

            double nominator = Math.abs(relX) + Math.abs(relY);
            if (nominator == 0) nominator = 1;

            double F_X = relX / nominator;
            double F_Y = relY / nominator;
            double F_heading = Range.clip(relativeAngleToTarget / Math.toRadians(30), -1, 1) * turnSpeed;

            double mov_X = (P_X + D_X + F_X) * movementSpeed;
            double mov_Y = (P_Y + D_Y + F_Y)  * movementSpeed;
            double mov_Heading = (P_heading + D_heading + F_heading) * movementSpeed;

            movement = new Pose2d(mov_X, mov_Y, mov_Heading);
        }

        return movement;
    }
}
