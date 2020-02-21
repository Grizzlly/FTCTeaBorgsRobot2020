package org.firstinspires.ftc.teamcode.MecanumProgramming;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;

@TeleOp(name="Mecanum_TeleOP", group="Mecanum")
//@Disabled
public class Mecanum_TeleOP extends OpMode
{
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor motorLift1 = null;
    private DcMotor motorLift2 = null;
    private DcMotor clawMainMotor = null;
    private DcMotor clawSecMotor = null;

    private Servo servo1;

    double surpress = 0.4;
    double powerUp = 0.0;
    double powerDown = 0.0;
    double pos = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init()
    {
        frontLeft  = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRight = hardwareMap.get(DcMotor.class, "fr_motor");
        backLeft  = hardwareMap.get(DcMotor.class, "bl_motor");
        backRight = hardwareMap.get(DcMotor.class, "br_motor");
        clawMainMotor  = hardwareMap.get(DcMotor.class, "clawMainMotor");
        clawSecMotor = hardwareMap.get(DcMotor.class, "clawSecMotor");
        servo1  = hardwareMap.get(Servo.class, "servo1");
        motorLift1 = hardwareMap.get(DcMotor.class, "motor_lift1");
        motorLift2= hardwareMap.get(DcMotor.class, "motor_lift2");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        motorLift1.setDirection(DcMotor.Direction.FORWARD);
        motorLift2.setDirection(DcMotor.Direction.REVERSE);
        clawSecMotor.setDirection(DcMotor.Direction.FORWARD);

        motorLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawMainMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        servo1.setPosition(0);
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
    public void loop() {
        double gamepadRightY = -gamepad2.right_stick_y;
        double gamepadLeftX = gamepad2.right_stick_x;
        double gamepadLeftTrigger = -gamepad2.left_trigger;
        double gamepadRightTrigger = gamepad2.right_trigger;

        powerUp = -gamepad1.left_trigger;
        powerDown = gamepad1.right_trigger;

        //gamepadRightY = gamepadRightY*gamepadRightY*Math.signum(gamepadRightY)/100;
        //gamepadLeftX = gamepadLeftX*gamepadLeftX*Math.signum(gamepadLeftX)/100;

        double powerFrontLeft = gamepadRightY + gamepadLeftX + (gamepadLeftTrigger + gamepadRightTrigger);
        double powerFrontRight = gamepadRightY - gamepadLeftX - (gamepadLeftTrigger + gamepadRightTrigger);
        double powerBackLeft = gamepadRightY - gamepadLeftX + (gamepadLeftTrigger + gamepadRightTrigger);
        double powerBackRight = gamepadRightY + gamepadLeftX - (gamepadLeftTrigger + gamepadRightTrigger);

        if (gamepad2.right_bumper) {
            powerFrontLeft *= surpress;
            powerFrontRight *= surpress;
            powerBackLeft *= surpress;
            powerBackRight *= surpress;
        }

        if(gamepad1.a)
            clawMainMotor.setPower(0.8);
        else if(gamepad1.y)
            clawMainMotor.setPower(-0.8);
        else
            clawMainMotor.setPower(0.0);

        if(gamepad1.left_bumper)
        {
            clawSecMotor.setPower(0.8);
        }
        else if(gamepad1.right_bumper)
        {
            clawSecMotor.setPower(-0.8);
        }
        else
            clawSecMotor.setPower(0.0);

        if(gamepad2.a)
            servo1.setPosition(0.9);
        if(gamepad2.y)
            servo1.setPosition(0);

        runtime.reset();
        //while(runtime.seconds() < 0.5);

        powerFrontLeft = Range.clip(powerFrontLeft, -1, 1);
        powerFrontRight = Range.clip(powerFrontRight, -1, 1);
        powerBackLeft = Range.clip(powerBackLeft, -1, 1);
        powerBackRight = Range.clip(powerBackRight, -1, 1);
        powerUp = Range.clip(powerUp, -1, 1);
        powerDown = Range.clip(powerDown, -1, 1);

        frontLeft.setPower(powerFrontLeft);
        frontRight.setPower(powerFrontRight);
        backLeft.setPower(powerBackLeft);
        backRight.setPower(powerBackRight);

        /*telemetry.addData("front left", frontLeft.getCurrentPosition());
        telemetry.addData("front right", frontRight.getCurrentPosition());
        telemetry.addData("back left", backLeft.getCurrentPosition());
        telemetry.addData("back right", backRight.getCurrentPosition());
        telemetry.addData("front left", frontLeft.getPower());
        telemetry.addData("front right", frontRight.getPower());
        telemetry.addData("back left", backLeft.getPower());
        telemetry.addData("back right", backRight.getPower());
        telemetry.addData("l1", motorLift1.getCurrentPosition());

         */

        if (gamepad1.left_trigger != 0.0)
        {
            motorLift1.setPower(powerUp);
            motorLift2.setPower(powerUp);
        } else if (gamepad1.right_trigger != 0.0)
        {
            motorLift1.setPower(powerDown);
            motorLift2.setPower(powerDown);
        }
        else
        {
            motorLift1.setPower(0.0);
            motorLift2.setPower(0.0);
        }

    }

    @Override
    public void stop()
    {

    }

}
