package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

@Autonomous(name= "Parcare_simpla", group="Parcare")
//@Disabled//comment out this line before using
public class Parcare_simpla extends LinearOpMode {
    private BNO055IMU gyro;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    public DcMotor clawSecMotor = null;
    public DcMotor clawMainMotor = null;
    private DcMotor motorLift1 = null;
    private DcMotor motorLift2 = null;

    double powerFL, powerFR, powerBL, powerBR;

    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = 0;
    private static int valLeft = 0;
    private static int valRight = 0;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {4f/8f+offsetX, 2f/8f+offsetY};
    private static float[] rightPos = {4f/8f+offsetX, 6f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    //definitieRobot func = new definitieRobot();

    @Override
    public void runOpMode() throws InterruptedException {

        gyro = hardwareMap.get(BNO055IMU.class, "giroscop");

        frontLeft = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRight = hardwareMap.get(DcMotor.class, "fr_motor");
        backLeft = hardwareMap.get(DcMotor.class, "bl_motor");
        backRight = hardwareMap.get(DcMotor.class, "br_motor");
        clawSecMotor = hardwareMap.get(DcMotor.class, "clawSecMotor");
        clawMainMotor = hardwareMap.get(DcMotor.class, "clawMainMotor");
        motorLift1 = hardwareMap.get(DcMotor.class, "motor_lift1");
        motorLift2= hardwareMap.get(DcMotor.class, "motor_lift2");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        clawSecMotor.setDirection(DcMotor.Direction.FORWARD);
        clawMainMotor.setDirection(DcMotor.Direction.REVERSE);
        motorLift1.setDirection(DcMotor.Direction.FORWARD);
        motorLift2.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        gyro.initialize(imuParameters);

        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            Acceleration gravity = gyro.getGravity();

            ///FACUT DIN OCHI <3
            driveFrontStraight(1500, 1500, 1500, 1500, 0, 0.9, 1);
            resetDrives();
            runtime.reset();
        }
    }

    public void driveBackStraight(int t1, int t2, int t3, int t4, double straightAngle, double speed, double mod)
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(Math.abs(frontRight.getCurrentPosition()) < t4-200)
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

            powerFL = -speed + ((error - straightAngle)/(100));
            powerFR = -speed - ((error - straightAngle)/(100));
            powerBL = -speed + ((error - straightAngle)/(100));
            powerBR = -speed - ((error - straightAngle)/(100));

            powerFL = Range.clip(powerFL, -1, 1);
            powerFR = Range.clip(powerFR, -1, 1);
            powerBL = Range.clip(powerBL, -1, 1);
            powerBR = Range.clip(powerBR, -1, 1);

            frontLeft.setPower(powerFL);
            frontRight.setPower(powerFR);
            backLeft.setPower(powerBL);
            backRight.setPower(powerBR);

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

    public void driveBackStraight_Lift(int t1, int t2, int t3, int t4, double straightAngle, double speed, double P)
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(Math.abs(frontRight.getCurrentPosition()) < t4-200)
        {
            motorLift1.setTargetPosition(0);

            Orientation angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double error = angle.firstAngle;


            frontLeft.setTargetPosition(t1);
            frontRight.setTargetPosition(t2);
            backLeft.setTargetPosition(t3);
            backRight.setTargetPosition(t4);

            powerFL = -speed + (error - straightAngle)/100;
            powerFR = -speed - (error - straightAngle)/100;
            powerBL = -speed + (error - straightAngle)/100;
            powerBR = -speed - (error - straightAngle)/100;

            powerFL = Range.clip(powerFL, -1, 1);
            powerFR = Range.clip(powerFR, -1, 1);
            powerBL = Range.clip(powerBL, -1, 1);
            powerBR = Range.clip(powerBR, -1, 1);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(powerFL);
            frontRight.setPower(powerFR);
            backLeft.setPower(powerBL);
            backRight.setPower(powerBR);
            motorLift1.setPower(-1);
            motorLift2.setPower(-1);

        }
        resetDrives();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        while(motorLift1.getCurrentPosition()>motorLift1.getTargetPosition()+20);
        motorLift1.setPower(0);
        motorLift2.setPower(0);
    }

    public void driveRightStraight(int t1, int t2, int t3, int t4, double straightAngle, double speed, int P)
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(Math.abs(frontRight.getCurrentPosition()) < t4-200)
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

            powerFL = -speed - (error - straightAngle)/100*P;
            powerFR =  speed + (error - straightAngle)/100*P;
            powerBL =  speed - (error - straightAngle)/100*P;
            powerBR = -speed + (error - straightAngle)/100*P;

            powerFL = Range.clip(powerFL, -1, 1);
            powerFR = Range.clip(powerFR, -1, 1);
            powerBL = Range.clip(powerBL, -1, 1);
            powerBR = Range.clip(powerBR, -1, 1);

            frontLeft.setPower(powerFL);
            frontRight.setPower(powerFR);
            backLeft.setPower(powerBL);
            backRight.setPower(powerBR);

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

    public void driveLeftStraight(int t1, int t2, int t3, int t4, double straightAngle, double speed, int P)
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(Math.abs(frontRight.getCurrentPosition()) < t4-200)
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

            powerFL =  speed - (error - straightAngle)/100*P;
            powerFR = -speed + (error - straightAngle)/100*P;
            powerBL = -speed - (error - straightAngle)/100*P;
            powerBR =  speed + (error - straightAngle)/100*P;

            powerFL = Range.clip(powerFL, -1, 1);
            powerFR = Range.clip(powerFR, -1, 1);
            powerBL = Range.clip(powerBL, -1, 1);
            powerBR = Range.clip(powerBR, -1, 1);

            frontLeft.setPower(powerFL);
            frontRight.setPower(powerFR);
            backLeft.setPower(powerBL);
            backRight.setPower(powerBR);

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

    public void driveFrontStraight(int t1, int t2, int t3, int t4, double straightAngle, double speed, int P)
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Orientation angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while(Math.abs(frontRight.getCurrentPosition()) < t4-200)
        {
            angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double error = angle.firstAngle;

            frontLeft.setTargetPosition(-t1);
            frontRight.setTargetPosition(-t2);
            backLeft.setTargetPosition(-t3);
            backRight.setTargetPosition(-t4);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            powerFL =  speed + (error - straightAngle)/100;
            powerFR =  speed - (error - straightAngle)/100;
            powerBL =  speed + (error - straightAngle)/100;
            powerBR =  speed - (error - straightAngle)/100;

            powerFL = Range.clip(powerFL, -1, 1);
            powerFR = Range.clip(powerFR, -1, 1);
            powerBL = Range.clip(powerBL, -1, 1);
            powerBR = Range.clip(powerBR, -1, 1);

            frontLeft.setPower(powerFL);
            frontRight.setPower(powerFR);
            backLeft.setPower(powerBL);
            backRight.setPower(powerBR);

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

    public void driveFrontStraight_Lift(int t1, int t2, int t3, int t4, double straightAngle, double speed, int P)
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Orientation angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while(Math.abs(frontRight.getCurrentPosition()) < t4-200)
        {
            motorLift1.setTargetPosition(800);

            angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double error = angle.firstAngle;

            frontLeft.setTargetPosition(-t1);
            frontRight.setTargetPosition(-t2);
            backLeft.setTargetPosition(-t3);
            backRight.setTargetPosition(-t4);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            powerFL =  speed + (error - straightAngle)/100;
            powerFR =  speed - (error - straightAngle)/100;
            powerBL =  speed + (error - straightAngle)/100;
            powerBR =  speed - (error - straightAngle)/100;

            powerFL = Range.clip(powerFL, -1, 1);
            powerFR = Range.clip(powerFR, -1, 1);
            powerBL = Range.clip(powerBL, -1, 1);
            powerBR = Range.clip(powerBR, -1, 1);

            frontLeft.setPower(powerFL);
            frontRight.setPower(powerFR);
            backLeft.setPower(powerBL);
            backRight.setPower(powerBR);
            motorLift1.setPower(1);
            motorLift2.setPower(1);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        while(motorLift1.getCurrentPosition() < motorLift1.getTargetPosition()-50);
        motorLift1.setPower(0);
        motorLift2.setPower(0);
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

        double error = straightAngle;
        if(error <= 0)
            while(Math.abs(error) < Math.abs(finishAngle+20))
            {
                Orientation angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                error = angle.firstAngle;

                frontLeft.setPower(-speed);
                frontRight.setPower(speed);
                backLeft.setPower(-speed);
                backRight.setPower(speed);
    
                /*telemetry.addData("Front Left", frontLeft.getCurrentPosition());
                telemetry.addData("Front Right", frontRight.getCurrentPosition());
                telemetry.addData("Back Left", backLeft.getCurrentPosition());
                telemetry.addData("Back Right", backRight.getCurrentPosition());
                telemetry.addData("Front Left", -0.4 - (error - straightAngle)/100);
                telemetry.addData("Front Right", 0.4 + (error - straightAngle)/100);
                telemetry.addData("Back Left", 0.4 - (error - straightAngle)/100);
                telemetry.addData("Back Right", -0.4 + (error - straightAngle)/100);
                telemetry.addData("Angle", error);
                telemetry.update();*/
            }
        else
            while(Math.abs(error) > Math.abs(finishAngle-20))
            {
                Orientation angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                error = angle.firstAngle;

                frontLeft.setPower(-speed);
                frontRight.setPower(speed);
                backLeft.setPower(-speed);
                backRight.setPower(speed);
    
                /*telemetry.addData("Front Left", frontLeft.getCurrentPosition());
                telemetry.addData("Front Right", frontRight.getCurrentPosition());
                telemetry.addData("Back Left", backLeft.getCurrentPosition());
                telemetry.addData("Back Right", backRight.getCurrentPosition());
                telemetry.addData("Front Left", -0.4 - (error - straightAngle)/100);
                telemetry.addData("Front Right", 0.4 + (error - straightAngle)/100);
                telemetry.addData("Back Left", 0.4 - (error - straightAngle)/100);
                telemetry.addData("Back Right", -0.4 + (error - straightAngle)/100);
                telemetry.addData("Angle", error);
                telemetry.update();*/
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

        double error = straightAngle;

        while(error < finishAngle-12)
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
