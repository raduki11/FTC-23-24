package org.firstinspires.ftc.teamcode.Teste;

import static org.firstinspires.ftc.teamcode.utils.Constants.Forward_Offset;
import static org.firstinspires.ftc.teamcode.utils.Constants.Horizontal_Offset;
import static org.firstinspires.ftc.teamcode.utils.Constants.X_Multiplier;
import static org.firstinspires.ftc.teamcode.utils.Constants.Y_Multiplier;
import static org.firstinspires.ftc.teamcode.utils.Mathematics.encoderTicksToCms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Algorithms.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.Algorithms.GVPathFollowing;
import org.firstinspires.ftc.teamcode.utils.Encoder;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

@Config
@Autonomous(name = "Test GuidingVector", group = "Autonomus")
public class Test_GuidingVector extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    private DcMotorEx RBM = null;
    private DcMotorEx LFM = null;
    private DcMotorEx RFM = null;
    private DcMotorEx LBM = null;
    private Encoder leftEnc = null;
    private Encoder rightEnc = null;
    private Encoder midEnc = null;
    private double Pos_X = 0;
    private double Pos_Y = 0;
    private double angle = 0;
    Vector2D current_point = new Vector2D(0, 0);
    GVPathFollowing drive = new GVPathFollowing();
    //Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    private double curr_LeftEncPos = 0;
    private double curr_RightEncPos = 0;
    private double curr_MidEncPos = 0;
    boolean isBusy = false;

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
        CubicBezierCurve traj = new CubicBezierCurve(new Vector2D(0,0),new Vector2D(0,100),new Vector2D(-4,100),new Vector2D(50,100));
        waitForStart();
        timer.reset();
        isBusy = true;
        Pos_X = 0;
        Pos_Y = 0;
        angle = 0;
        current_point = new Vector2D(0, 0);
        while (opModeIsActive()) {
            updateOdometryPos();
            Vector2D movSpeed = drive.calcGuidanceVector(traj,current_point);
            setPowers(movSpeed);
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
        double prev_LeftEncPos = curr_LeftEncPos;
        double prev_RightEncPos = curr_RightEncPos;
        double prev_MidEncPos = curr_MidEncPos;
        //Localization with position
        citireEncodere();

        double delta_LeftEncPos = curr_LeftEncPos - prev_LeftEncPos;
        double delta_RightEncPos = curr_RightEncPos - prev_RightEncPos;
        double delta_MidEncPos = curr_MidEncPos - prev_MidEncPos;

        double dtheta = (delta_RightEncPos - delta_LeftEncPos) / Horizontal_Offset;
        double dx = (delta_LeftEncPos + delta_RightEncPos) / 2.0;
        double dy = delta_MidEncPos - dtheta * Forward_Offset;

        Pos_X += dx * Math.cos(angle) - dy * Math.sin(angle);
        Pos_Y += dy * Math.cos(angle) + dx * Math.sin(angle);
        angle += dtheta;

        current_point = new Vector2D(Pos_Y, Pos_X);
    }

    private void TelemetryPos() {
        telemetry.addData("Pozitie pe OX fata de start:", current_point.getX());
        telemetry.addData("Pozitie pe OY fata de start:", current_point.getY());
        telemetry.addData("Unghi rotire:", (Math.toDegrees(current_point.getHeading())));
        telemetry.update();
    }

    private void setPowers(Vector2D mov) {
        double LFM_pow = mov.getX() - mov.getY() - mov.getHeading();
        double LBM_pow = mov.getX() + mov.getY() - mov.getHeading();
        double RBM_pow = mov.getX() - mov.getY() + mov.getHeading();
        double RFM_pow = mov.getX() + mov.getY() + mov.getHeading();
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