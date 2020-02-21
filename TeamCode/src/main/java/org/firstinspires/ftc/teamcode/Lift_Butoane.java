package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Lift_Butoane", group="Linear Opmode")
//@Disabled
public class Lift_Butoane extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

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

    int k=0;
    int n=0;
    boolean pressedUp = false;
    boolean pressedDown = false;

    int ticks = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        frontLeft  = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRight = hardwareMap.get(DcMotor.class, "fr_motor");
        backLeft  = hardwareMap.get(DcMotor.class, "bl_motor");
        backRight = hardwareMap.get(DcMotor.class, "br_motor");
        motorLift1 = hardwareMap.get(DcMotor.class, "motor_lift1");
        motorLift2= hardwareMap.get(DcMotor.class, "motor_lift2");
        clawMainMotor  = hardwareMap.get(DcMotor.class, "clawMainMotor");
        clawSecMotor = hardwareMap.get(DcMotor.class, "clawSecMotor");
        servo1  = hardwareMap.get(Servo.class, "servo1");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        motorLift1.setDirection(DcMotor.Direction.FORWARD);
        motorLift2.setDirection(DcMotor.Direction.REVERSE);
        clawSecMotor.setDirection(DcMotor.Direction.FORWARD);

        motorLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        servo1.setPosition(0);

        double  omniSurpress = 0.4;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            //omni
            double gamepadRightY = -gamepad2.right_stick_y;
            double gamepadLeftX = gamepad2.right_stick_x;
            double gamepadLeftTrigger = -gamepad2.left_trigger;
            double gamepadRightTrigger = gamepad2.right_trigger;

            powerUp = -gamepad1.left_trigger;
            powerDown = gamepad1.right_trigger;

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

            if (gamepad1.left_trigger != 0.0)
            {
                motorLift1.setPower(powerUp);
                motorLift2.setPower(powerUp);
            }
            else if (gamepad1.right_trigger != 0.0)
            {
                motorLift1.setPower(powerDown);
                motorLift2.setPower(powerDown);
            }
            else
            {
                motorLift1.setPower(0.0);
                motorLift2.setPower(0.0);
            }

            pressedUp = false;
            if(gamepad1.dpad_up && !pressedUp)
            {
                if(k == 5)
                    k = 0;
                k++;
                n=k;
                pressedUp = true;
                LiftGoUpTo(k);
            }
            telemetry.addData("sa", motorLift1.getCurrentPosition());
            telemetry.update();
            pressedDown = false;
            if(gamepad1.b && !pressedDown)
            {
                n--;
                pressedDown = true;
                k = n;
                LiftGoDown(n);
            }

            pressedDown = false;
            if(gamepad1.dpad_down && !pressedDown)
            {
                pressedDown = true;
                n=0;
                LiftGoDown(n);
            }
            telemetry.update();
        }
    }

    public void LiftGoUpTo(int k)
    {
        if(k==0)
            motorLift1.setTargetPosition(0);
        else if(k==1)
            motorLift1.setTargetPosition(1000);
        else
            motorLift1.setTargetPosition(k*1000);

        motorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLift1.setPower(1);
        motorLift2.setPower(1);

        motorLift1.getCurrentPosition();
        while (motorLift1.getCurrentPosition() < motorLift1.getTargetPosition()-50);
        {
            double gamepadRightY = -gamepad2.right_stick_y;
            double gamepadLeftX = gamepad2.right_stick_x;
            double gamepadLeftTrigger = -gamepad2.left_trigger;
            double gamepadRightTrigger = gamepad2.right_trigger;

            powerUp = -gamepad1.left_trigger;
            powerDown = gamepad1.right_trigger;

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
                clawMainMotor.setPower(0.5);
            else if(gamepad1.y)
                clawMainMotor.setPower(-0.5);
            else
                clawMainMotor.setPower(0.0);

            powerFrontLeft = Range.clip(powerFrontLeft, -1, 1);
            powerFrontRight = Range.clip(powerFrontRight, -1, 1);
            powerBackLeft = Range.clip(powerBackLeft, -1, 1);
            powerBackRight = Range.clip(powerBackRight, -1, 1);

            frontLeft.setPower(powerFrontLeft);
            frontRight.setPower(powerFrontRight);
            backLeft.setPower(powerBackLeft);
            backRight.setPower(powerBackRight);

            telemetry.addData("saa", motorLift1.getCurrentPosition());
            telemetry.update();
        }

        motorLift1.setPower(0);
        motorLift2.setPower(0);
    }

    public void LiftGoDown(int n)
    {
        if(n==0)
            motorLift1.setTargetPosition(0);
        else if(n==1)
            motorLift1.setTargetPosition(1000);
        else
            motorLift1.setTargetPosition(n*1000);

        motorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLift1.setPower(-1);
        motorLift2.setPower(-1);


        while(motorLift1.getCurrentPosition() > motorLift1.getTargetPosition()+50)
        {
            double gamepadRightY = -gamepad2.right_stick_y;
            double gamepadLeftX = gamepad2.right_stick_x;
            double gamepadLeftTrigger = -gamepad2.left_trigger;
            double gamepadRightTrigger = gamepad2.right_trigger;

            powerUp = -gamepad1.left_trigger;
            powerDown = gamepad1.right_trigger;

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
                clawMainMotor.setPower(0.5);
            else if(gamepad1.y)
                clawMainMotor.setPower(-0.5);
            else
                clawMainMotor.setPower(0.0);

            powerFrontLeft = Range.clip(powerFrontLeft, -1, 1);
            powerFrontRight = Range.clip(powerFrontRight, -1, 1);
            powerBackLeft = Range.clip(powerBackLeft, -1, 1);
            powerBackRight = Range.clip(powerBackRight, -1, 1);

            frontLeft.setPower(powerFrontLeft);
            frontRight.setPower(powerFrontRight);
            backLeft.setPower(powerBackLeft);
            backRight.setPower(powerBackRight);
        }

        motorLift1.setPower(0);
        motorLift2.setPower(0);
    }
}
