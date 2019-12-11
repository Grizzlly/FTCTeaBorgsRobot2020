package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareRobot
{
    //private enum RunMode
    //{
    //    POWER,
    //    ENCODER
    //}

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor motorLift;

    double circumferinta = Math.PI*10;
    double rotatii;
    int DrivingTarget;

    public DcMotor clawMainMotor;
    //public DcMotor clawSecMotor;

    public void initFromMap(HardwareMap hardwareMap)
    {
        frontLeft = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRight = hardwareMap.get(DcMotor.class, "fr_motor");
        backLeft = hardwareMap.get(DcMotor.class, "bl_motor");
        backRight = hardwareMap.get(DcMotor.class, "br_motor");

        motorLift = hardwareMap.get(DcMotor.class, "motor_lift");

        clawMainMotor  = hardwareMap.get(DcMotor.class, "clawMainMotor");
        //clawSecMotor = hardwareMap.get(DcMotor.class, "clawSecMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        motorLift.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawMainMotor.setDirection(DcMotor.Direction.FORWARD);
        clawMainMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //clawSecMotor.setDirection(DcMotor.Direction.FORWARD);

        stopReset();
    }

    private void stopReset()
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void stopResetLift()
    {
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunToPos()
    {
        //switch(mode)
        //{
        //    case POWER:
        //
        //}
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //frontLeft.get
    }

    public boolean isBusy()
    {
        return (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy());
    }

    public boolean isBusy(int targetPos)
    {
        return !(targetPos<=10+Math.abs(frontLeft.getCurrentPosition()));
    }

    public void moveForward(double power)
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void moveForward(double power, int distanta)
    {
        rotatii = distanta/circumferinta;
        DrivingTarget = (int)(rotatii*1120);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopReset();
        setRunToPos();

        frontLeft.setTargetPosition(-DrivingTarget);
        frontRight.setTargetPosition(-DrivingTarget);
        backLeft.setTargetPosition(-DrivingTarget);
        backRight.setTargetPosition(-DrivingTarget);
        setRunToPos();

        moveForward(power);
    }

    public void moveBack(double power, int distanta)
    {
        rotatii = distanta/circumferinta;
        DrivingTarget = (int)(rotatii*1120);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopReset();
        setRunToPos();

        frontLeft.setTargetPosition(-DrivingTarget);
        frontRight.setTargetPosition(-DrivingTarget);
        backLeft.setTargetPosition(-DrivingTarget);
        backRight.setTargetPosition(-DrivingTarget);
        setRunToPos();

        moveBackwards(power);
    }

    //public void moveForward(double distance, double time)

    public void moveBackwards(double power)
    {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
    }

    public void moveLeft(double power)
    {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void moveRight(double power)
    {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    public void rotateRight(double power)
    {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void rotateLeft(double power)
    {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    public void stopRobot()
    {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    public void liftUp(double power)
    {
        motorLift.setPower(power);
    }

    public void liftUp(int rot)
    {
        stopReset();
    }

    public void move(double powerFR, double powerFL, double powerBR, double powerBL)
    {
        frontLeft.setPower(powerFL);
        frontRight.setPower(powerFR);
        backLeft.setPower(powerBL);
        backRight.setPower(powerBR);
    }

    /*public void apucaTava()
    {
        clawSecMotor.setPower(-0.5);
    }

    public void stopTava()
    {
        clawSecMotor.setPower(0);
    }

    public void lasaTava()
    {
        clawSecMotor.setPower(0.5);
    }
    */
    public void moveLift(double power)
    {
        motorLift.setPower(power);
    }

    public void grabStone(boolean grab)
    {
        if(grab) clawMainMotor.setPower(1);
        else clawMainMotor.setPower(-1);
    }
}
