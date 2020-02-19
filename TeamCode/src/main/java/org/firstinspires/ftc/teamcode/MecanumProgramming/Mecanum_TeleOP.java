package org.firstinspires.ftc.teamcode.MecanumProgramming;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;

@TeleOp(name="Mecanum_TeleOP", group="Mecanum")
//@Disabled
public class Mecanum_TeleOP extends OpMode
{
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor motorLift = null;
    private DcMotor clawMainMotor = null;
    private DcMotor clawSecMotor = null;

    double surpress = 0.4;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init()
    {
        frontLeft  = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRight = hardwareMap.get(DcMotor.class, "fr_motor");
        backLeft  = hardwareMap.get(DcMotor.class, "bl_motor");
        backRight = hardwareMap.get(DcMotor.class, "br_motor");
        motorLift = hardwareMap.get(DcMotor.class, "motor_lift");
        clawMainMotor  = hardwareMap.get(DcMotor.class, "clawMainMotor");
        clawSecMotor = hardwareMap.get(DcMotor.class, "clawSecMotor");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        clawSecMotor.setDirection(DcMotor.Direction.FORWARD);

        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawMainMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        double gamepadRightY = -gamepad2.right_stick_y*100/127;
        double gamepadLeftX = gamepad2.right_stick_x*100/127;
        double gamepadLeftTrigger = -gamepad2.left_trigger;
        double gamepadRightTrigger = gamepad2.right_trigger;

        gamepadRightY = gamepadRightY*gamepadRightY*Math.signum(gamepadRightY)/100;
        gamepadLeftX = gamepadLeftX*gamepadLeftX*Math.signum(gamepadLeftX)/100;

        double powerFrontLeft   = gamepadRightY + gamepadLeftX + (gamepadLeftTrigger + gamepadRightTrigger);
        double powerFrontRight  = gamepadRightY - gamepadLeftX - (gamepadLeftTrigger + gamepadRightTrigger);
        double powerBackLeft    = gamepadRightY - gamepadLeftX + (gamepadLeftTrigger + gamepadRightTrigger);
        double powerBackRight   = gamepadRightY + gamepadLeftX - (gamepadLeftTrigger + gamepadRightTrigger);

        if(gamepad2.right_bumper)
        {
            powerFrontLeft  *= surpress;
            powerFrontRight *= surpress;
            powerBackLeft   *= surpress;
            powerBackRight  *= surpress;
        }

        powerFrontLeft  = Range.clip(powerFrontLeft,  -1, 1);
        powerFrontRight = Range.clip(powerFrontRight, -1, 1);
        powerBackLeft   = Range.clip(powerBackLeft,   -1, 1);
        powerBackRight  = Range.clip(powerBackRight,  -1, 1);

        frontLeft.setPower(powerFrontLeft);
        frontRight.setPower(powerFrontRight);
        backLeft.setPower(powerBackLeft);
        backRight.setPower(powerBackRight);


    }

    @Override
    public void stop()
    {

    }

}