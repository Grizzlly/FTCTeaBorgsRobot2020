package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.NoSuchElementException;

public class AutonomousFunctions
{
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    private ElapsedTime Time = new ElapsedTime();

    private HardwareMap hardwareMap;

    public AutonomousFunctions(HardwareMap hwMap)
    {
        hardwareMap = hwMap;
        frontLeft = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRight = hardwareMap.get(DcMotor.class, "fr_motor");
        backLeft = hardwareMap.get(DcMotor.class, "bl_motor");
        backRight = hardwareMap.get(DcMotor.class, "br_motor");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void Move_Forward(double power)
    {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void Move_Backwards(double power)
    {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
    }

    public void Move_Left(double power)
    {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void Move_Right(double power)
    {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    public void Rotate_Right(double power)
    {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void Rotate_Left(double power)
    {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    public void Stop_Robot()
    {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }
    public void PicaTava() throws NoSuchMethodException
    {
        throw new NoSuchMethodException();

    }
}
