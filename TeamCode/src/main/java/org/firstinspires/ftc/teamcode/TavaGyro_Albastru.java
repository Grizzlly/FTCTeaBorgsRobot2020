package org.firstinspires.ftc.teamcode;
//cazul 3 stanga
import android.view.Gravity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "TavaGyro_Albastru", group = "tava")
public class TavaGyro_Albastru extends LinearOpMode {

    private BNO055IMU gyro;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    public DcMotor clawSecMotor = null;
    public DcMotor clawMainMotor = null;

    double error;

    public static HardwareMap hwMap;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        gyro = hardwareMap.get(BNO055IMU.class, "giroscop");

        frontLeft  = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRight = hardwareMap.get(DcMotor.class, "fr_motor");
        backLeft  = hardwareMap.get(DcMotor.class, "bl_motor");
        backRight = hardwareMap.get(DcMotor.class, "br_motor");
        clawSecMotor = hardwareMap.get(DcMotor.class, "clawSecMotor");
        clawMainMotor = hardwareMap.get(DcMotor.class, "clawMainMotor");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        clawSecMotor.setDirection(DcMotor.Direction.FORWARD);
        clawMainMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Create new IMU Parameters object.
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        gyro.initialize(imuParameters);
        // Prompt user to press start buton.

        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
            // Put run blocks here.
            // Get absolute orientation
            // Get acceleration due to force of gravity.
            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            Acceleration gravity = gyro.getGravity();


            driveLeftStraight(3300, 3300, 3300, 3300, 0, 0.4);
            runtime.reset();
            while(runtime.seconds()<0.5){}
            resetDrives();

            clawSecMotor.setPower(-1);
            runtime.reset();
            while(runtime.seconds()<0.3){}

            driveRightStraight(3000, 3000, 3000, 3000, 0, 0.4);
            runtime.reset();
            while(runtime.seconds()<0.5){}
            resetDrives();

            driveFrontStraight(1500, 1500, 1500, 1500, 0, 0.4);
            runtime.reset();
            while(runtime.seconds()<0.5){}
            resetDrives();

            RotateLeft(0, 0.4, 90);
            resetDrives();

            driveLeftStraight(2500, 2500, 2500, 2500, 90, 0.4);
            runtime.reset();
            while(runtime.seconds()<0.5){}
            resetDrives();

            clawSecMotor.setPower(0.6);
            runtime.reset();
            while(runtime.seconds()<0.3){}
            clawSecMotor.setPower(0);

            driveRightStraight(500, 500, 500, 500, 90, 0.4);
            runtime.reset();
            while(runtime.seconds()<0.5){}
            resetDrives();

            driveBackStraight(2000, 2000, 2000, 2000, 90, 0.4);
            runtime.reset();
            while(runtime.seconds()<0.5){}
            resetDrives();

            driveRightStraight(3000, 3000, 3000, 3000, 90, 0.4);
            runtime.reset();
            while(runtime.seconds()<0.5){}
            resetDrives();
        }
    }

    public void driveBackStraight(int t1, int t2, int t3, int t4, double straightAngle, double speed)
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(backRight.getCurrentPosition() < t4-200)
        {
            Orientation angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double error = angle.firstAngle;

            frontLeft.setTargetPosition(t1);
            frontRight.setTargetPosition(t2);
            backLeft.setTargetPosition(t3);
            backRight.setTargetPosition(t4);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(-speed + (error - straightAngle)/100);
            frontRight.setPower(-speed - (error - straightAngle)/100);
            backLeft.setPower(-speed + (error - straightAngle)/100);
            backRight.setPower(-speed - (error - straightAngle)/100);

            telemetry.addData("Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", frontRight.getCurrentPosition());
            telemetry.addData("Back Left", backLeft.getCurrentPosition());
            telemetry.addData("Back Right", backRight.getCurrentPosition());
            telemetry.addData("Front Left", -0.4 - (error - straightAngle)/100);
            telemetry.addData("Front Right", 0.4 + (error - straightAngle)/100);
            telemetry.addData("Back Left", 0.4 - (error - straightAngle)/100);
            telemetry.addData("Back Right", -0.4 + (error - straightAngle)/100);
            telemetry.addData("Angle", error);
            telemetry.update();
        }
        resetDrives();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void driveRightStraight(int t1, int t2, int t3, int t4, double straightAngle, double speed)
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(backRight.getCurrentPosition() > -t4+200)
        {
            Orientation angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double error = angle.firstAngle;

            frontLeft.setTargetPosition(-t1);
            frontRight.setTargetPosition(t2);
            backLeft.setTargetPosition(t3);
            backRight.setTargetPosition(-t4);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(-speed - (error - straightAngle)/100);
            frontRight.setPower(speed + (error - straightAngle)/100);
            backLeft.setPower(speed - (error - straightAngle)/100);
            backRight.setPower(-speed + (error - straightAngle)/100);

            telemetry.addData("Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", frontRight.getCurrentPosition());
            telemetry.addData("Back Left", backLeft.getCurrentPosition());
            telemetry.addData("Back Right", backRight.getCurrentPosition());
            telemetry.addData("Front Left", -0.4 - (error - straightAngle)/100);
            telemetry.addData("Front Right", 0.4 + (error - straightAngle)/100);
            telemetry.addData("Back Left", 0.4 - (error - straightAngle)/100);
            telemetry.addData("Back Right", -0.4 + (error - straightAngle)/100);
            telemetry.addData("Angle", error);
            telemetry.update();
        }
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    public void driveLeftStraight(int t1, int t2, int t3, int t4, double straightAngle, double speed)
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(backRight.getCurrentPosition() < t4-200)
        {
            Orientation angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double error = angle.firstAngle;

            frontLeft.setTargetPosition(t1);
            frontRight.setTargetPosition(-t2);
            backLeft.setTargetPosition(-t3);
            backRight.setTargetPosition(t4);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(speed - (error - straightAngle)/100);
            frontRight.setPower(-speed + (error - straightAngle)/100);
            backLeft.setPower(-speed - (error - straightAngle)/100);
            backRight.setPower(speed + (error - straightAngle)/100);

            telemetry.addData("Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", frontRight.getCurrentPosition());
            telemetry.addData("Back Left", backLeft.getCurrentPosition());
            telemetry.addData("Back Right", backRight.getCurrentPosition());
            telemetry.addData("Front Left", -0.4 - (error - straightAngle)/100);
            telemetry.addData("Front Right", 0.4 + (error - straightAngle)/100);
            telemetry.addData("Back Left", 0.4 - (error - straightAngle)/100);
            telemetry.addData("Back Right", -0.4 + (error - straightAngle)/100);
            telemetry.addData("Angle", error);
            telemetry.update();
        }
        resetDrives();
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
    }

    public void driveFrontStraight(int t1, int t2, int t3, int t4, double straightAngle, double speed)
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(backRight.getCurrentPosition() > -t4+200)
        {
            Orientation angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double error = angle.firstAngle;

            frontLeft.setTargetPosition(-t1);
            frontRight.setTargetPosition(-t2);
            backLeft.setTargetPosition(-t3);
            backRight.setTargetPosition(-t4);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(speed + (error - straightAngle)/100);
            frontRight.setPower(speed - (error - straightAngle)/100);
            backLeft.setPower(speed + (error - straightAngle)/100);
            backRight.setPower(speed - (error - straightAngle)/100);

            telemetry.addData("Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", frontRight.getCurrentPosition());
            telemetry.addData("Back Left", backLeft.getCurrentPosition());
            telemetry.addData("Back Right", backRight.getCurrentPosition());
            telemetry.addData("Front Left", -0.4 - (error - straightAngle)/100);
            telemetry.addData("Front Right", 0.4 + (error - straightAngle)/100);
            telemetry.addData("Back Left", 0.4 - (error - straightAngle)/100);
            telemetry.addData("Back Right", -0.4 + (error - straightAngle)/100);
            telemetry.addData("Angle", error);
            telemetry.update();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void RotateRight(double straightAngle, double speed, double finishAngle)
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        error = straightAngle;

        while(error > finishAngle+3)
        {
            Orientation angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = angle.firstAngle;

            frontLeft.setPower(-speed);
            frontRight.setPower(speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);

            telemetry.addData("Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", frontRight.getCurrentPosition());
            telemetry.addData("Back Left", backLeft.getCurrentPosition());
            telemetry.addData("Back Right", backRight.getCurrentPosition());
            telemetry.addData("Front Left", -0.4 - (error - straightAngle)/100);
            telemetry.addData("Front Right", 0.4 + (error - straightAngle)/100);
            telemetry.addData("Back Left", 0.4 - (error - straightAngle)/100);
            telemetry.addData("Back Right", -0.4 + (error - straightAngle)/100);
            telemetry.addData("Angle", error);
            telemetry.update();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void RotateLeft(double straightAngle, double speed, double finishAngle)
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        error = straightAngle;

        while(error < finishAngle-3)
        {
            Orientation angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = angle.firstAngle;

            frontLeft.setPower(speed);
            frontRight.setPower(-speed);
            backLeft.setPower(speed);
            backRight.setPower(-speed);

            telemetry.addData("Front Left", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", frontRight.getCurrentPosition());
            telemetry.addData("Back Left", backLeft.getCurrentPosition());
            telemetry.addData("Back Right", backRight.getCurrentPosition());
            telemetry.addData("Front Left", -0.4 - (error - straightAngle)/100);
            telemetry.addData("Front Right", 0.4 + (error - straightAngle)/100);
            telemetry.addData("Back Left", 0.4 - (error - straightAngle)/100);
            telemetry.addData("Back Right", -0.4 + (error - straightAngle)/100);
            telemetry.addData("Angle", error);
            telemetry.update();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    void resetDrives() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void WAIT() {
        while (!(backRight.getTargetPosition()-100 >= backRight.getCurrentPosition() && backRight.getCurrentPosition() >= backRight.getTargetPosition()+100)) {
            telemetry.addData("", frontLeft.getCurrentPosition());
            telemetry.addData("", frontRight.getCurrentPosition());
            telemetry.addData("", backLeft.getCurrentPosition());
            telemetry.addData("", backRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
