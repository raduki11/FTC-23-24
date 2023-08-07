package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.BezierCurveGenerator.CubicCurve;
import static utils.Constants.Forward_Offset;
import static utils.Constants.Horizontal_Offset;
import static utils.Constants.X_Multiplier;
import static utils.Constants.Y_Multiplier;
import static utils.Constants.kD_Heading;
import static utils.Constants.kD_X;
import static utils.Constants.kD_Y;
import static utils.Constants.kP_Headng;
import static utils.Constants.kP_X;
import static utils.Constants.kP_Y;
import static utils.Mathematics.AngleWrap;
import static utils.Mathematics.encoderTicksToCms;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@Config
@Autonomous(name = "Test capacitate", group = "Autonomus")
public class Autonomie_Rr_Custom extends LinearOpMode {
    ElapsedTime test = new ElapsedTime();
    private double prev_error_POS = 0, prev_error_heading = 0, curr_time = 0, prev_time = 0, sumI_pos = 0, sumI_heading = 0, prev_error_X = 0, prev_error_Y = 0;
    private double const_pow = 0;
    ElapsedTime timer = new ElapsedTime();
    private DcMotorEx RBM = null;
    private DcMotorEx LFM = null;
    private DcMotorEx RFM = null;
    private DcMotorEx LBM = null;
    private Encoder leftEnc = null,
            rightEnc = null,
            midEnc = null;
    private double Pos_X = 0;
    private double Pos_Y = 0;
    private double angle = 0;
    Pose2d current_point = new Pose2d(0, 0, 0);
    //Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    private double curr_LeftEncPos = 0,
            curr_RightEncPos = 0,
            curr_MidEncPos = 0;

    private double prev_LeftEncPos = 0,
            prev_RightEncPos = 0,
            prev_MidEncPos = 0;

    private double delta_LeftEncPos = 0,
            delta_RightEncPos = 0,
            delta_MidEncPos = 0;
    private double r0 = 0, r1 = 0;

    private int wayPoint = 0;
    int go = 1;
    boolean isBusy = false;
    boolean isClose = false;
    boolean isDone = false;
    boolean Done = false;
    int da = -1;
    Pose2d target = new Pose2d();

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        LBM = hardwareMap.get(DcMotorEx.class, "LBM");
        RBM = hardwareMap.get(DcMotorEx.class, "RBM");
        LFM = hardwareMap.get(DcMotorEx.class, "LFM");
        RFM = hardwareMap.get(DcMotorEx.class, "RFM");

        //Reverse la motoare
        RFM.setDirection(DcMotorEx.Direction.REVERSE);
        RBM.setDirection(DcMotorEx.Direction.REVERSE);


        LBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "RBM"));
        rightEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "RFM"));
        midEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "LBM"));

        leftEnc.setDirection(Encoder.Direction.FORWARD);
        rightEnc.setDirection(Encoder.Direction.FORWARD);
        midEnc.setDirection(Encoder.Direction.FORWARD);

        //START

        Point A = new Point(0, 0);
        Point B = new Point(0, 100);
        Point C = new Point(-4, 100);
        Point D = new Point(50, 100);
        List<Pose2d> traj = CubicCurve(A, B, C, D, 10);
        waitForStart();
        timer.reset();
        isBusy = true;
        Pos_X = 0;
        Pos_Y = 0;
        angle = 0;
        current_point = new Pose2d(0, 0, 0);
        while (opModeIsActive()) {
            updateOdometryPos();
            //if(!Done)PathRunner(traj);
            /**
             if(da==-1)
             goToPoint(new Pose2d(100,0,0));
             else
             if(da == 1)
             goToPoint(new Pose2d(0,0,0));
             if(!isBusy)
             {
             isBusy = true;
             da=-da;
             sleep(500);
             test.reset();
             }
             */
            goToPoint(new Pose2d(100, 0, Math.toRadians(0)), 0.2, 0);
            TelemetryPos();
        }
    }

    private void citireEncodere() {
        double LeftPos = leftEnc.getCurrentPosition();
        double RightPos = rightEnc.getCurrentPosition();
        double MidPos = midEnc.getCurrentPosition();

        curr_LeftEncPos = encoderTicksToCms(LeftPos) * X_Multiplier;
        curr_RightEncPos = encoderTicksToCms(RightPos) * X_Multiplier;
        curr_MidEncPos = encoderTicksToCms(MidPos) * Y_Multiplier;
    }

    private void updateOdometryPos() {
        prev_LeftEncPos = curr_LeftEncPos;
        prev_RightEncPos = curr_RightEncPos;
        prev_MidEncPos = curr_MidEncPos;
        //Localization with position
        citireEncodere();

        delta_LeftEncPos = curr_LeftEncPos - prev_LeftEncPos;
        delta_RightEncPos = curr_RightEncPos - prev_RightEncPos;
        delta_MidEncPos = curr_MidEncPos - prev_MidEncPos;

        double dtheta = (delta_RightEncPos - delta_LeftEncPos) / Horizontal_Offset;
        double dx = (delta_LeftEncPos + delta_RightEncPos) / 2.0;
        double dy = delta_MidEncPos - dtheta * Forward_Offset;

        //double theta = angle + (dtheta / 2.0);
        Pos_X += dx * Math.cos(angle) - dy * Math.sin(angle);
        Pos_Y += dy * Math.cos(angle) + dx * Math.sin(angle);
        angle += dtheta;
        current_point = new Pose2d(Pos_X, Pos_Y, angle);
    }

    private void PathRunner(List<Pose2d> Trajectory) {

    }

    private void goToPoint(Pose2d target_point, double movementSpeed, double turnSpeed) {
        updateOdometryPos();

        Pose2d err = target_point.minus(current_point);
        double dist = Math.hypot(err.getX(), err.getY());
        if (dist <= 1)
            setPowers(0, 0, 0, 0);
        else {
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
            double rel_Turn = relativeAngleToTarget + target_point.getHeading() - Math.toRadians(90);

            double nominator = Math.abs(relX) + Math.abs(relY);

            double F_X = relX / nominator * movementSpeed;
            double F_Y = relY / nominator * movementSpeed;
            double F_heading = Range.clip(rel_Turn / Math.toRadians(30), -1, 1) * turnSpeed;

            double mov_X = P_X + D_X + F_X;
            double mov_Y = P_Y + D_Y + F_Y;
            double mov_Heading = P_heading + D_heading + F_heading;

            double LFM_pow = mov_X - mov_Y - mov_Heading;
            double LBM_pow = mov_X + mov_Y - mov_Heading;
            double RBM_pow = mov_X - mov_Y + mov_Heading;
            double RFM_pow = mov_X + mov_Y + mov_Heading;

            setPowers(LFM_pow, LBM_pow, RFM_pow, RBM_pow);
        }
    }

    private void TelemetryPos() {
        telemetry.addData("Pozitie pe OX fata de start:", current_point.getX());
        telemetry.addData("Pozitie pe OY fata de start:", current_point.getY());
        telemetry.addData("Unghi rotire:", (Math.toDegrees(current_point.getHeading())));
        telemetry.update();
    }

    public void setPowers(double LFM_pow, double LBM_pow, double RFM_pow, double RBM_pow) {
        double max_pow = Math.max(Math.max(RFM_pow, RBM_pow), Math.max(LFM_pow, LBM_pow));
        if (max_pow == 0)
            max_pow = 1;
        LFM.setPower(LFM_pow / max_pow);
        RFM.setPower(RFM_pow / max_pow);
        LBM.setPower(LBM_pow / max_pow);
        RBM.setPower(RBM_pow / max_pow);
    }
}

/**
 * if (Math.abs(curr_error_POS) < 10)
 * sumI_pos += curr_error_POS * (curr_time-prev_time);
 * if (Math.abs(curr_error_heading) < 2)
 * sumI_pos += curr_error_heading * (curr_time-prev_time);
 * double P_pos = kP_pos * curr_error_POS;
 * double P_heading = kP_pos * curr_error_heading;
 * double I_pos = kI_pos * sumI_pos;                                                                        ///PID position
 * double I_heading = kI_pos * sumI_pos;
 * double D_pos = kD_pos * (curr_error_POS - prev_error_POS)/(curr_time - prev_time);
 * double D_heading = kD_pos * (curr_error_heading - prev_error_heading)/(curr_time - prev_time);
 * double correction_Pos = P_pos + I_pos + D_pos;
 * double correction_heading = P_heading + I_heading + D_heading;
 * prev_error_POS = curr_error_POS;
 * prev_error_POS = curr_error_heading;
 * prev_time = curr_time;
 */
/**
 * double err_x = target_point.getX() - current_point.getX(),
 * err_y = target_point.getY() - current_point.getY(),
 * err_theta = target_point.getHeading() - current_point.getHeading();
 * double theta = current_point.getHeading();
 * double ex = Math.cos(theta) * err_x + Math.sin(theta) * err_y,
 * ey = Math.cos(theta) * err_y - Math.sin(theta) * err_x;
 * double k = 2 * kZeta * Math.sqrt(wd * wd + vd * vd);
 * double since0;
 * if(err_theta==0)since0 = Math.sin(err_theta);
 * else
 * since0 = Math.sin(err_theta) / err_theta;
 * double vc = vd * Math.cos(err_theta) + k * ex;
 * double w = wd + k * err_theta + kBeta * vd * since0 * ey;
 * <p>
 * double vLeft = vc - w * rB;
 * double vRight = vc + w * rB;
 */