package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement.followCurve;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
@Disabled
public class MyOpMode extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init()
    {

    }

    @Override
    public void init_loop()
    {

    }

    @Override
    public void start()
    {
        runtime.reset();
    }

    @Override
    public void loop()
    {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0, 0, 1, 1, 20, Math.toRadians(50), 1));
        allPoints.add(new CurvePoint(75, 260, 1, 10, 20, Math.toRadians(50), 1));
        allPoints.add(new CurvePoint(280, 260, 1, 1, 20, Math.toRadians(50), 1));
        allPoints.add(new CurvePoint(280, 50, 1, 1, 20, Math.toRadians(50), 1));
        allPoints.add(new CurvePoint(180, 0, 1, 1, 20, Math.toRadians(50), 1));
        allPoints.add(new CurvePoint(179, 179, 1, 1, 20, Math.toRadians(50), 1));

        followCurve(allPoints, Math.toRadians(90));
    }

    @Override
    public void stop()
    {

    }

}
