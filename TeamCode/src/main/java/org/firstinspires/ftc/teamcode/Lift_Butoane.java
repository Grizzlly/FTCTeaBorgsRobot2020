package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="Linear Opmode")
@Disabled
public class Lift_Butoane extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor motorLift = null;
    DcMotorController controler;

    private DcMotor clawMainMotor = null;
    private DcMotor clawSecMotor = null;

    private final double MaxRots = 5000;

    private final int CPR = 1112;
    boolean reached = false;

    int k=0;
    int n=0;
    boolean pressedUp = false;
    boolean pressedDown = false;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeft  = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRight = hardwareMap.get(DcMotor.class, "fr_motor");
        backLeft  = hardwareMap.get(DcMotor.class, "bl_motor");
        backRight = hardwareMap.get(DcMotor.class, "br_motor");
        motorLift = hardwareMap.get(DcMotor.class, "motor_lift");
        clawMainMotor  = hardwareMap.get(DcMotor.class, "clawMainMotor");
        clawSecMotor = hardwareMap.get(DcMotor.class, "clawSecMotor");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        clawSecMotor.setDirection(DcMotor.Direction.FORWARD);
        clawMainMotor.setDirection(DcMotor.Direction.REVERSE);

        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawMainMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double  omniSurpress = 0.4;

        waitForStart();
        while (opModeIsActive()) {
            //omni
            double gamepadRightY = -gamepad2.right_stick_y*100/127;
            double gamepadLeftX = gamepad2.right_stick_x*100/127;
            double gamepadLeftTrigger = -gamepad2.left_trigger;
            double gamepadRightTrigger = gamepad2.right_trigger;

            gamepadRightY = gamepadRightY*gamepadRightY*Math.signum(gamepadRightY)/100;
            gamepadLeftX = gamepadLeftX*gamepadLeftX*Math.signum(gamepadLeftX)/100;
            
            double powerFrontLeft = -gamepadRightY - gamepadLeftX - (gamepadLeftTrigger + gamepadRightTrigger);
            double powerFrontRight = gamepadRightY - gamepadLeftX - (gamepadLeftTrigger + gamepadRightTrigger);
            double powerBackLeft = -gamepadRightY + gamepadLeftX - (gamepadLeftTrigger + gamepadRightTrigger);
            double powerBackRight = gamepadRightY + gamepadLeftX - (gamepadLeftTrigger + gamepadRightTrigger);

            double powerLatchingUp = -gamepad1.right_trigger;
            double powerLatchingDown = gamepad1.left_trigger;

            powerFrontLeft = Range.clip(powerFrontLeft, -1, 1);
            powerFrontRight = Range.clip(powerFrontRight, -1, 1);
            powerBackLeft = Range.clip(powerBackLeft, -1, 1);
            powerBackRight = Range.clip(powerBackRight, -1, 1);
            powerLatchingUp = Range.clip(powerLatchingUp, -1, 1);
            powerLatchingDown = Range.clip(powerLatchingDown, -1, 1);

            pressedUp = false;
            if(gamepad1.x && !pressedUp)
            {
                k++;
                pressedUp = true;
                LiftGoUpTo(k);
            }

            n=k;
            pressedDown = false;
            if(gamepad1.b && !pressedDown)
            {
                n--;
                LiftGoDown(n);
            }

            if(gamepad1.y==true)
            {
                clawMainMotor.setPower(0.5);
            }
            else if(gamepad1.a==true)
            {
                clawMainMotor.setPower(-0.5);
            }
            else {
                clawMainMotor.setPower(0.0);
            }

            /**if (gamepad2.right_bumper == true) {
                powerFrontLeft = powerFrontLeft *   omniSurpress;
                powerBackLeft = powerBackLeft *     omniSurpress;
                powerFrontRight = powerFrontRight * omniSurpress;
                powerBackRight = powerBackRight *   omniSurpress;

            }

            frontLeft.setPower(powerFrontLeft);
            frontRight.setPower(powerFrontRight);
            backLeft.setPower(powerBackLeft);
            backRight.setPower(powerBackRight);
            //omni

            //lift
            if(motorLift.getCurrentPosition()<=CPR/5.5) powerLatchingUp=0;
            //if(motorLift.getCurrentPosition()>=5400) powerLatchingDown=0;

            if((powerLatchingUp==0 && powerLatchingDown==0))
            {
                motorLift.setTargetPosition(motorLift.getCurrentPosition());
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLift.setPower(1);
            }
            else motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(powerLatchingUp !=0)
                motorLift.setPower(powerLatchingUp);
            else if(powerLatchingDown == 0)
                motorLift.setPower(0);
            if(powerLatchingDown !=0)
                motorLift.setPower(powerLatchingDown);
            else if(powerLatchingUp == 0)
                motorLift.setPower(0);

            //claws
            if(gamepad1.y==true)
            {
                clawMainMotor.setPower(-0.5);
            }
            else if(gamepad1.a==true)
            {
                clawMainMotor.setPower(0.5);
            }
            else {
                clawMainMotor.setPower(0.0);
            }

            if(gamepad1.dpad_down==true)
            {
                clawSecMotor.setPower(-0.5);
            }
            else if(gamepad1.dpad_up==true)
            {
                clawSecMotor.setPower(0.5);
            }
            else {
                clawSecMotor.setPower(0.0);
            }
            //claws

            if(gamepad2.x) {
                frontLeft.setPower(0.0);
                frontRight.setPower(0.0);
                backLeft.setPower(0.0);
                backRight.setPower(0.0);
                motorLift.setPower(0.0);

                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                clawSecMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                clawSecMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
             **/

            telemetry.update();
        }
    }

    public void LiftGoUpTo(int k)
    {
        if(k==0)
            motorLift.setTargetPosition(0);
        else if(k==1)
            motorLift.setTargetPosition(1000);
        else
            motorLift.setTargetPosition(k*1000);

        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setPower(1.0);
        while (motorLift.getCurrentPosition() < motorLift.getTargetPosition()-50);
        motorLift.setPower(0.0);
    }

    public void LiftGoDown(int n)
    {
        if(n==0)
            motorLift.setTargetPosition(0);
        else if(n==1)
            motorLift.setTargetPosition(1000);
        else
            motorLift.setTargetPosition(n*1000);

        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setPower(1.0);
        while(motorLift.getCurrentPosition() > motorLift.getTargetPosition()+50);
        motorLift.setPower(0);
    }
}