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

import org.firstinspires.ftc.teamcode.Algorithms.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.Algorithms.GVFPathFollowing;
import org.firstinspires.ftc.teamcode.utils.Encoder;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

import java.util.List;

@TeleOp(name = "Test GuidingVector", group = "Autonomus")
public class Test_GuidedVectorField extends LinearOpMode {
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
    Pose2d curr_point = new Pose2d();
    GVFPathFollowing drive = new GVFPathFollowing();
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
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //START
        CubicBezierCurve traj = new CubicBezierCurve(new Vector2D(0, 0), new Vector2D(48.5, -82.7), new Vector2D(75.6, 65), new Vector2D(105.3, -19.4));
        waitForStart();


        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        timer.reset();
        isBusy = true;
        Pos_X = 0;
        Pos_Y = 0;
        angle = 0;
        current_point = new Vector2D(0, 0);
        curr_point = new Pose2d(0, 0, 0);
        Pose2d target_point = new Pose2d(traj.calculate(1).getX(), traj.calculate(1).getY(), traj.heading(1));
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            updateOdometryPos();
            Pose2d movVector = drive.calculateGuidanceVector(traj, current_point, curr_point.getHeading());
            Pose2d err = target_point.minus(curr_point);
            if (Math.abs(err.getX()) > 1 || Math.abs(err.getY()) > 1 || Math.abs(Math.toDegrees(err.getHeading())) > 2)
                setPowers(movVector, 0.5);
            else
                setPowers(movVector, 0);
            telemetry.addData("Heading final:", Math.toDegrees(traj.heading(1)));
            telemetry.addData("Vit OX:", movVector.getX());
            telemetry.addData("Vit OY:", movVector.getY());
            telemetry.addData("Vit rotire:", (movVector.getHeading()));
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

        current_point = new Vector2D(Pos_X, Pos_Y);
        curr_point = new Pose2d(Pos_X, Pos_Y, angle);
    }

    private void TelemetryPos() {
        telemetry.addData("Pozitie pe OX fata de start:", curr_point.getX());
        telemetry.addData("Pozitie pe OY fata de start:", curr_point.getY());
        telemetry.addData("Unghi rotire:", (Math.toDegrees(curr_point.getHeading())));
        telemetry.update();
    }

    private void setPowers(Pose2d mov, double movementSpeed) {
        double LFM_pow = mov.getX() - mov.getY() - mov.getHeading();
        double LBM_pow = mov.getX() + mov.getY() - mov.getHeading();
        double RBM_pow = mov.getX() - mov.getY() + mov.getHeading();
        double RFM_pow = mov.getX() + mov.getY() + mov.getHeading();

        // Range.clip(LFM_pow, -0.8, 0.8);
        // Range.clip(LBM_pow, -0.8, 0.8);
        //Range.clip(RFM_pow, -0.8, 0.8);
        // Range.clip(RBM_pow, -0.8, 0.8);

        LFM.setPower(LFM_pow * movementSpeed);
        RFM.setPower(RFM_pow * movementSpeed);
        LBM.setPower(LBM_pow * movementSpeed);
        RBM.setPower(RBM_pow * movementSpeed);

        telemetry.addData("LFM power", LFM_pow * movementSpeed);
        telemetry.addData("RFM power", RFM_pow * movementSpeed);
        telemetry.addData("LBM power", LBM_pow * movementSpeed);
        telemetry.addData("RBM power", RBM_pow * movementSpeed);
    }
}