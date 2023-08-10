package Teste;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

import z_NotInUse_OldStuff.BezierCurveGenerator;
import z_NotInUse_OldStuff.Point;

@Disabled
@TeleOp(name = "Curve test")
public class BezierCurveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Point A = new Point(0, 0);
        Point B = new Point(0, 100);
        Point C = new Point(-4, 100);
        Point D = new Point(50, 100);
        List<Pose2d> Traj = new ArrayList<>();
        Traj = BezierCurveGenerator.CubicCurve(A, B, C, D, 10);
        telemetry.addLine("Gata");
        telemetry.update();
        waitForStart();
        for (int i = 0; i < Traj.size(); i++) {
            telemetry.addData("Puncte:", i);
            telemetry.addData("Puncte:", Traj.get(i).getX());
            telemetry.addData("Puncte:", Traj.get(i).getY());
            telemetry.addData("Puncte:", Math.toDegrees(Traj.get(i).getHeading()));
            telemetry.update();
            sleep(1000);
        }

        /**
         for(double i = 0; i <= NumberOfPoints; i++)
         {
         t = i / NumberOfPoints;
         P.X = (1-t) * (1-t) * (1-t) * A.X + 3 * (1-t) * (1-t) * t * B.X + 3 * (1-t) * t * t * C.X + t * t * t * D.X;
         P.Y = (1-t) * (1-t) * (1-t) * A.Y + 3 * (1-t) * (1-t) * t * B.Y + 3 * (1-t) * t * t * C.Y + t * t * t * D.Y;
         //P.heading = calculateHeading(P0, P1, P2, P3, t);
         //curvePoints.add(i, P);
         telemetry.addData("Puncte:",i);
         telemetry.addData("X:",P.X);
         telemetry.addData("Y:",P.Y);
         telemetry.addData("Heading:",P.heading);
         telemetry.update();
         sleep(10000);
         }
         */
    }
}
