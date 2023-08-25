package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.utils.Constants.Forward_Offset;
import static org.firstinspires.ftc.teamcode.utils.Constants.Horizontal_Offset;
import static org.firstinspires.ftc.teamcode.utils.Constants.X_Multiplier;
import static org.firstinspires.ftc.teamcode.utils.Constants.Y_Multiplier;
import static org.firstinspires.ftc.teamcode.utils.Mathematics.encoderTicksToCms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Algorithms.GoToPoint;
import org.firstinspires.ftc.teamcode.utils.Encoder;

import java.util.List;

@TeleOp(name = "Test goToPoint", group = "Autonomus")
public class Test_goToPoint extends LinearOpMode {
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
    Pose2d current_point = new Pose2d(0, 0, 0);
    GoToPoint drive = new GoToPoint();
    //Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    private double curr_LeftEncPos = 0;
    private double curr_RightEncPos = 0;
    private double curr_MidEncPos = 0;
    private boolean isBusy = false;
    private boolean manual = false;

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

        LBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "RBM"));
        rightEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "RFM"));
        midEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "LBM"));

        leftEnc.setDirection(Encoder.Direction.REVERSE);
        rightEnc.setDirection(Encoder.Direction.FORWARD);
        midEnc.setDirection(Encoder.Direction.REVERSE);

        //START
        int da = -1;
        waitForStart();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        timer.reset();
        isBusy = true;
        Pos_X = 0;
        Pos_Y = 0;
        angle = 0;
        Pose2d target_point = new Pose2d(150, 0, Math.toRadians(0));
        current_point = new Pose2d(0, 0, 0);
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            updateOdometryPos();

            if (gamepad1.a)
                manual = true;
            if (gamepad1.b)
                manual = false;

            TelemetryPos();

            if (!manual) {
                //if(!Done)PathRunner(traj);
                if (!isBusy) {
                    if (da == -1) target_point = new Pose2d(150, 0, Math.toRadians(0));
                    else target_point = new Pose2d(0, 0, Math.toRadians(0));
                    isBusy = true;
                    da = -da;
                    sleep(500);
                }
                Pose2d err = target_point.minus(current_point);
                if (Math.abs(err.getX()) > 2 || Math.abs(err.getY()) > 2 || Math.toDegrees(Math.abs(err.getHeading())) > 2) {
                    setPowers(drive.goToPoint(timer, current_point, target_point, 0.4, 0), 0.6);
                } else {
                    setPowers(drive.goToPoint(timer, current_point, target_point, 0, 0), 0);
                }
            } else {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                double denominator = Math.abs(Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.4));

                LFM.setPower((y + x + rx) / denominator);
                LBM.setPower((y - x + rx) / denominator);
                RFM.setPower((y - x - rx) / denominator);
                RBM.setPower((y + x - rx) / denominator);
                if (gamepad1.y)
                    resetPos();
            }
        }
    }

    private void resetPos() {
        LBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pos_X = 0;
        Pos_Y = 0;
        angle = 0;
        current_point = new Pose2d(0, 0, 0);
        curr_LeftEncPos = 0;
        curr_RightEncPos = 0;
        curr_MidEncPos = 0;
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

        current_point = new Pose2d(Pos_X, Pos_Y, angle);
    }

    private void TelemetryPos() {
        telemetry.addData("Pozitie pe OX fata de start:", current_point.getX());
        telemetry.addData("Pozitie pe OY fata de start:", current_point.getY());
        telemetry.addData("Unghi rotire:", (Math.toDegrees(current_point.getHeading())));
        telemetry.update();
    }

    private void setPowers(Pose2d movmentPower, double speed) {
        double mov_X = movmentPower.getX();
        double mov_Y = movmentPower.getY();
        double mov_Heading = movmentPower.getHeading();

        double rot_X = mov_Y * Math.cos(angle) + mov_X * Math.cos(angle);// -mov_Y * Math.sin(-angle) + mov_X * Math.cos(-angle);
        double rot_Y = mov_Y * Math.cos(angle) - mov_X * Math.sin(angle);//

        double denominator = Math.abs(Math.max(Math.abs(rot_Y) + Math.abs(rot_X) + Math.abs(mov_Heading), 1));

        double LFM_pow = (rot_X + rot_Y - mov_Heading) / denominator;
        double LBM_pow = (rot_X - rot_Y - mov_Heading) / denominator;
        double RBM_pow = (rot_X - rot_Y + mov_Heading) / denominator;
        double RFM_pow = (rot_X + rot_Y + mov_Heading) / denominator;

        //double LFM_pow = mov_X - mov_Y - mov_Heading;
        //double LBM_pow = mov_X + mov_Y - mov_Heading;
        //double RBM_pow = mov_X - mov_Y + mov_Heading;
        //double RFM_pow = mov_X + mov_Y + mov_Heading;


        LFM.setPower(LFM_pow * speed);
        RFM.setPower(RFM_pow * speed);
        LBM.setPower(LBM_pow * speed);
        RBM.setPower(RBM_pow * speed);

        telemetry.addData("LFM power", LFM_pow * speed);
        telemetry.addData("RFM power", RFM_pow * speed);
        telemetry.addData("LBM power", LBM_pow * speed);
        telemetry.addData("RBM power", RBM_pow * speed);
        telemetry.addData("movx: ", mov_X);
        telemetry.addData("movy: ", mov_Y);
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