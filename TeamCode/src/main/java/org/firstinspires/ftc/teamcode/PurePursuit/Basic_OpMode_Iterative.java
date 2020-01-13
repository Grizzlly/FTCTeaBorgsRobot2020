package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
//@Disabled
public class Basic_OpMode_Iterative extends OpMode
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
        RobotMovement.goToPosition(358/2, 358/2, 0.3);
    }

    @Override
    public void stop()
    {

    }

}
