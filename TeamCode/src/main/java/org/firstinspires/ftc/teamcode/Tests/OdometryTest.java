package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Algorithms.OdometryLocalizer;

import java.util.List;

@TeleOp(name="Test Odometrie", group="Iterative Opmode")
public class OdometryTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Motoare
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        OdometryLocalizer odo = new OdometryLocalizer(hardwareMap);
        odo.Motor_init();
        //START
        waitForStart();
        //LOOP
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        while(!isStopRequested())
        {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            Pose2d currPos = odo.getCurrentPosition();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            odo.setPowers(y + x + rx,y - x - rx,y - x + rx,y + x - rx);

            if(gamepad1.a)
                odo.resetPos();

            telemetry.addData("Pozitie pe OX fata de start:",currPos.getX());
            telemetry.addData("Pozitie pe OY fata de start:",currPos.getY());
            telemetry.addData("Unghi rotire:",(Math.toDegrees(currPos.getHeading())));
            telemetry.update();
        }
    }

/**
    private double RealAngle(double angle)
    {
        if(angle>=360||angle<=-360)angle=0;
        return angle;
    }
 */
}

