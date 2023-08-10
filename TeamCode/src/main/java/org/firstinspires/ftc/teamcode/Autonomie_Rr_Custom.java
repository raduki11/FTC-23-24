package org.firstinspires.ftc.teamcode;

import static utils.Constants.Forward_Offset;
import static utils.Constants.Horizontal_Offset;
import static utils.Constants.X_Multiplier;
import static utils.Constants.Y_Multiplier;
import static utils.Mathematics.encoderTicksToCms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import utils.Encoder;

@Config
@Autonomous(name = "Test capacitate", group = "Autonomus")
public class Autonomie_Rr_Custom extends LinearOpMode {
    public  double prev_error_POS = 0, prev_error_heading = 0, curr_time = 0, prev_time = 0, sumI_pos = 0, sumI_heading = 0, prev_error_X = 0, prev_error_Y = 0;
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
    GoToPoint drive = new GoToPoint();
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
            setPowers(drive.goToPoint(timer, new Pose2d(100, 0, Math.toRadians(0)), current_point, 0.2, 0));
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

    private void TelemetryPos() {
        telemetry.addData("Pozitie pe OX fata de start:", current_point.getX());
        telemetry.addData("Pozitie pe OY fata de start:", current_point.getY());
        telemetry.addData("Unghi rotire:", (Math.toDegrees(current_point.getHeading())));
        telemetry.update();
    }

    public void setPowers(List<Double> MotorPower) {
        double LFM_pow = MotorPower.get(0);
        double LBM_pow = MotorPower.get(1);
        double RFM_pow = MotorPower.get(2);
        double RBM_pow = MotorPower.get(3);
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
 * <p>
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
 * <p>
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
 * <p>
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