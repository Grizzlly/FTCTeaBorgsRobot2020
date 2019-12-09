package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

public class definitieRobot {
    /* Public OpMode members. */
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor latchingLeft = null;
    public DcMotor latchingRight = null;
    public DcMotor cup = null;
    //////////////////////////////////////////////////////

    /* local OpMode members. */
    public static HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();
    private static int CPR = 1112;     //1112 counts per revolution

    /* Constructor */
    public definitieRobot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft = hwMap.get(DcMotor.class, "fl_motor");
        frontRight = hwMap.get(DcMotor.class, "fr_motor");
        backLeft = hwMap.get(DcMotor.class, "bl_motor");
        backRight = hwMap.get(DcMotor.class, "br_motor");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    void move_back(double power, int rots) {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(rots);
        frontRight.setTargetPosition(rots);
        backLeft.setTargetPosition(rots);
        backRight.setTargetPosition(rots);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);


    }

    void move_front(double power, int rots) {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(-rots);
        frontRight.setTargetPosition(-rots);
        backLeft.setTargetPosition(-rots);
        backRight.setTargetPosition(-rots);

        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);

    }

    void move_right(double power, int rots) {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(-rots);
        frontRight.setTargetPosition(rots);
        backLeft.setTargetPosition(rots);
        backRight.setTargetPosition(-rots);

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);


    }

    void move_left(double power, int rots) {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        frontLeft.setTargetPosition(rots);
        frontRight.setTargetPosition(-rots);
        backLeft.setTargetPosition(-rots);
        backRight.setTargetPosition(rots);

        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    void move_front_right(double power, int rots) {
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setTargetPosition(-rots);
        backLeft.setTargetPosition(-rots);

        frontRight.setPower(power);
        backLeft.setPower(power);
    }

    void move_front_left(double power, int rots) {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(-rots);
        backRight.setTargetPosition(-rots);

        frontLeft.setPower(power);
        backRight.setPower(power);
    }

    void move_back_left(double power, int rots) {


        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setTargetPosition(-rots);
        backLeft.setTargetPosition(-rots);

        frontRight.setPower(-power);
        backLeft.setPower(-power);
    }

    void move_back_right(double power, int rots) {

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(-rots);
        backRight.setTargetPosition(-rots);

        frontLeft.setPower(-power);
        backRight.setPower(-power);
    }

    void rotate_right(double power, int rots) {

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(-rots);
        frontRight.setTargetPosition(rots);
        backLeft.setTargetPosition(-rots);
        backRight.setTargetPosition(rots);

        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);

    }

    void rotate_left(double power, int rots) {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(rots);
        frontRight.setTargetPosition(-rots);
        backLeft.setTargetPosition(rots);
        backRight.setTargetPosition(-rots);

        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);

    }

    void resetDrives() {

        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    boolean isBusy() {
        if (frontLeft.isBusy()) return true;
        if (frontRight.isBusy()) return true;
        if (backLeft.isBusy()) return true;
        if (backRight.isBusy()) return true;
        return false;
    }
}