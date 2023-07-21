package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.BezierCurveGenerator.CubicCurve;

import static utils.Constants.BASE;
import static utils.Constants.Forward_Offset;
import static utils.Constants.Horizontal_Offset;
import static utils.Constants.WHEEL_RADIUS;
import static utils.Constants.X_Multiplier;
import static utils.Constants.Y_Multiplier;
import static utils.Mathematics.encoderTicksToCms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;



import java.util.List;

@Config
@Autonomous(name="Test capacitate",group="Autonomus")
public class Autonomie_Rr_Custom extends LinearOpMode
{
    public static double        kP_posX = 0.3, kI_posX=0, kD_posX = 0.08,
                                kP_posY = 0.3, kI_posY=0, kD_posY = 0.08,
                                kP_heading = 0.05, kI_heading = 0, kD_heading = 0;  //ToDo coeficienti PID

    private double prev_error_POS = 0, prev_error_heading = 0, curr_time = 0, prev_time = 0, sumI_pos = 0, sumI_heading = 0,prev_error_X = 0, prev_error_Y = 0;
    private double const_pow = 0.1;
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
    private double Field_X = 0;
    private double Field_Y = 0;
    Pose2d current_point = new Pose2d(0,0,0);
    private double  curr_LeftEncPos =0,
            curr_RightEncPos = 0,
            curr_MidEncPos = 0;

    private  double     prev_LeftEncPos = 0,
            prev_RightEncPos = 0,
            prev_MidEncPos = 0;

    private  double     delta_LeftEncPos = 0,
            delta_RightEncPos = 0,
            delta_MidEncPos = 0;
    private double r0 = 0, r1 = 0;

    private int  wayPoint = 0;
    int go = 1;
    boolean isBusy = false;
    @Override
    public void runOpMode() throws InterruptedException {
        LBM = hardwareMap.get(DcMotorEx.class, "LBM");
        RBM = hardwareMap.get(DcMotorEx.class, "RBM");
        LFM  = hardwareMap.get(DcMotorEx.class, "LFM");
        RFM = hardwareMap.get(DcMotorEx.class, "RFM");

        //Reverse la motoare
        RFM.setDirection(DcMotorEx.Direction.REVERSE);
        RBM.setDirection(DcMotorEx.Direction.REVERSE);

        /**
         LBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         LFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         RFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         RBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         */


        leftEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "RBM"));
        rightEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "RFM"));
        midEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "LBM"));

        leftEnc.setDirection(Encoder.Direction.FORWARD);
        rightEnc.setDirection(Encoder.Direction.FORWARD);
        midEnc.setDirection(Encoder.Direction.FORWARD);

        leftEnc.reset();
        rightEnc.reset();
        midEnc.reset();
        Pos_X = 0;
        Pos_Y = 0;
        angle = 0;
        //START

        Point A = new Point(0 , 0);
        Point B = new Point(0, 100);
        Point C = new Point(-4, 100);
        Point D = new Point(50, 100);
        List<Pose2d> traj= CubicCurve(A, B, C, D,10);
        waitForStart();
        timer.reset();
        isBusy = true;
        while(opModeIsActive())
        {
            updateOdometryPos();
            goToPoint(new Pose2d(100,0,Math.toRadians(0)));
            TelemetryPos();
        }
    }
    private void citireEncodere()
    {
        double LeftPos = leftEnc.getCurrentPosition();
        double RightPos = rightEnc.getCurrentPosition();
        double MidPos = midEnc.getCurrentPosition();

        curr_LeftEncPos = encoderTicksToCms(LeftPos) * X_Multiplier;
        curr_RightEncPos = encoderTicksToCms(RightPos) * X_Multiplier;
        curr_MidEncPos = encoderTicksToCms(MidPos) * Y_Multiplier;
    }

    private void updateOdometryPos()
    {
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
        current_point = new Pose2d(Pos_X,Pos_Y,angle);
    }
    private void PathRunner(List<Pose2d> Trajectory)
    {
        if(isBusy) {
            Pose2d pct = Trajectory.get(wayPoint).minus(current_point);
            if (pct.getX() <= 4 && pct.getY() <= 4 && pct.getHeading() <= 1)
                wayPoint++;
            if (wayPoint >= Trajectory.size()) {
                isBusy = false;
                wayPoint = 0;
            } else
                goToPoint(Trajectory.get(wayPoint));
        }
    }
    private void goToPoint(Pose2d target_point)
    {
        updateOdometryPos();

        double curr_error_heading = target_point.getHeading() - current_point.getHeading(); // distance between actual si target
        double error_X = target_point.getX() - current_point.getX();
        double error_Y = target_point.getY() - current_point.getY();

        double P_X = kP_posX * error_X;
        double P_Y = kP_posY * error_Y;
        double P_heading = kP_heading * curr_error_heading;

        curr_time = timer.seconds();
        double D_X = kD_posX * (error_X - prev_error_X)/(curr_time - prev_time);
        double D_Y = kD_posY * (error_Y - prev_error_Y)/(curr_time - prev_time);
        double D_heading = kD_heading * (curr_error_heading - prev_error_heading)/(curr_time - prev_time);

        double correction_X = P_X + D_X;
        double correction_Y = P_Y + D_Y;
        double correction_heading = P_heading + D_heading;

        prev_error_X = error_X;
        prev_error_Y = error_Y;
        prev_error_heading = curr_error_heading;
        prev_time = curr_time;

        double  vFL = correction_X - correction_Y - (BASE * correction_heading),
                vBL = correction_X + correction_Y - (BASE * correction_heading),
                vBR = correction_X - correction_Y + (BASE * correction_heading),
                vFR = correction_X + correction_Y + (BASE * correction_heading);
        LFM.setPower(const_pow + vFL/WHEEL_RADIUS);
        LBM.setPower(const_pow + vBL/WHEEL_RADIUS);
        RFM.setPower(const_pow + vFR/WHEEL_RADIUS);
        RBM.setPower(const_pow + vBR/WHEEL_RADIUS);
    }
    void TelemetryPos()
    {
        telemetry.addData("Pozitie pe OX fata de start:",Pos_X);
        telemetry.addData("Pozitie pe OY fata de start:",Pos_Y);
        telemetry.addData("Unghi rotire:",(Math.toDegrees(angle)));
        telemetry.update();
    }
}
/**
 if (Math.abs(curr_error_POS) < 10)
 sumI_pos += curr_error_POS * (curr_time-prev_time);
 if (Math.abs(curr_error_heading) < 2)
 sumI_pos += curr_error_heading * (curr_time-prev_time);
 double P_pos = kP_pos * curr_error_POS;
 double P_heading = kP_pos * curr_error_heading;
 double I_pos = kI_pos * sumI_pos;                                                                        ///PID position
 double I_heading = kI_pos * sumI_pos;
 double D_pos = kD_pos * (curr_error_POS - prev_error_POS)/(curr_time - prev_time);
 double D_heading = kD_pos * (curr_error_heading - prev_error_heading)/(curr_time - prev_time);
 double correction_Pos = P_pos + I_pos + D_pos;
 double correction_heading = P_heading + I_heading + D_heading;
 prev_error_POS = curr_error_POS;
 prev_error_POS = curr_error_heading;
 prev_time = curr_time;
 */