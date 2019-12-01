package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareRobot
{
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor motorLift;

    public DcMotor clawMainMotor;
    public DcMotor clawSecMotor;

    public void initFromMap(HardwareMap hardwareMap)
    {
        frontLeft = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRight = hardwareMap.get(DcMotor.class, "fr_motor");
        backLeft = hardwareMap.get(DcMotor.class, "bl_motor");
        backRight = hardwareMap.get(DcMotor.class, "br_motor");

        motorLift = hardwareMap.get(DcMotor.class, "motor_lift");

        clawMainMotor  = hardwareMap.get(DcMotor.class, "clawMainMotor");
        clawSecMotor = hardwareMap.get(DcMotor.class, "clawSecMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        motorLift.setDirection(DcMotor.Direction.FORWARD);

        clawMainMotor.setDirection(DcMotor.Direction.FORWARD);
        clawSecMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void moveForward(double power)
    {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

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

    public void apucaTava()
    {
        clawSecMotor.setPower(1);
    }

    public void lasaTava()
    {
        clawSecMotor.setPower(-1);
    }

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
