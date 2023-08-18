package org.firstinspires.ftc.teamcode.Test;

import static org.firstinspires.ftc.teamcode.utils.Constants.Forward_Offset;
import static org.firstinspires.ftc.teamcode.utils.Constants.Horizontal_Offset;
import static org.firstinspires.ftc.teamcode.utils.Constants.X_Multiplier;
import static org.firstinspires.ftc.teamcode.utils.Constants.Y_Multiplier;
import static org.firstinspires.ftc.teamcode.utils.Mathematics.encoderTicksToCms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utils.Encoder;

@TeleOp(name="Pozitie", group="Iterative Opmode")
public class OdometryTest extends LinearOpMode {
    private double      Pos_X = 0,
                        Pos_Y =0,
                        heading = 0;
    private DcMotorEx RBM = null;
    private DcMotorEx LFM = null;
    private DcMotorEx RFM = null;
    private DcMotorEx LBM = null;

    //Telemetry telemetry= FtcDashboard.getInstance().getTelemetry();

    private Encoder leftEnc = null,
            rightEnc = null,
            midEnc = null;

    private double angle = 0;
    private double  r0 =0,
                    r1 = 0;

    private double  curr_LeftEncPos =0,
            curr_RightEncPos = 0,
            curr_MidEncPos = 0;

    private  double     prev_LeftEncPos = 0,
            prev_RightEncPos = 0,
            prev_MidEncPos = 0;

    private  double     delta_LeftEncPos = 0,
            delta_RightEncPos = 0,
            delta_MidEncPos = 0;
    private double  Pos_X_last = 0;
    private double Pos_Y_last = 0;

    BNO055IMU imu ;
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException {
        //Motoare
        LBM = hardwareMap.get(DcMotorEx.class, "LBM");
        RBM = hardwareMap.get(DcMotorEx.class, "RBM");
        LFM  = hardwareMap.get(DcMotorEx.class, "LFM");
        RFM = hardwareMap.get(DcMotorEx.class, "RFM");

        //Reverse la motoare
        RFM.setDirection(DcMotorEx.Direction.REVERSE);
        RBM.setDirection(DcMotorEx.Direction.REVERSE);

        LBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "BNo055IMUCalibration.json";
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "RBM"));
        rightEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "RFM"));
        midEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "LBM"));

        leftEnc.setDirection(Encoder.Direction.FORWARD);
        rightEnc.setDirection(Encoder.Direction.FORWARD);
        midEnc.setDirection(Encoder.Direction.REVERSE);

        //START

        waitForStart();
        //LOOP
        while(!isStopRequested())
        {
            updateOdometryPos();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            LFM.setPower(y + x + rx);
            LBM.setPower(y - x + rx);
            RFM.setPower(y - x - rx);
            RBM.setPower(y + x - rx);

            if(gamepad1.a)
                resetPos();

            telemetry.addData("Pozitie pe OX fata de start:",Pos_X);
            telemetry.addData("Pozitie pe OY fata de start:",Pos_Y);
            telemetry.addData("Unghi rotire:",(Math.toDegrees(angle)));
            telemetry.update();
        }
    }
    private void resetPos()
    {
        LBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pos_X_last = 0;
        Pos_Y_last = 0;
        Pos_X = 0;
        Pos_Y = 0;
        angle = 0;
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
/**
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

        //Calcul deplasare
        double delta_heading = (delta_LeftEncPos - delta_RightEncPos) / Horizontal_Offset;
        double forward = (delta_RightEncPos + delta_LeftEncPos) / 2.0;
        double strafe = delta_MidEncPos - delta_heading * Forward_Offset;

        if(delta_heading == 0) {
            r0 = forward / 1;
            r1 = strafe / 1;
        }
        else
        {
            r0 = forward / delta_heading;
            r1 = strafe / delta_heading;
        }
        double rel_x = r0 * Math.sin(delta_heading) - r1 * (1- Math.cos(delta_heading));
        double rel_y = r1 * Math.sin(delta_heading) + r0 * (1- Math.cos(delta_heading));

        angles =  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        Pos_X += rel_x * Math.cos(angles.firstAngle) - rel_y * Math.sin(angles.firstAngle);
        Pos_Y += rel_y * Math.cos(angles.firstAngle) + rel_x * Math.sin(angles.firstAngle);
        //angle += delta_heading;
    }
    */

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
        Pos_X += (dx * Math.cos(angle) - dy * Math.sin(angle));
        Pos_Y += (dy * Math.cos(angle) + dx * Math.sin(angle));
        angle += dtheta;
    }

/**
    private double RealAngle(double angle)
    {
        if(angle>=360||angle<=-360)angle=0;
        return angle;
    }
 */
}

