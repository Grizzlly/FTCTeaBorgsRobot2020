package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "DetectorAlbastru_Gyro", group="Sky autonomous")
//@Disabled//comment out this line before using
public class DetectorAlbastru_Gyro extends LinearOpMode {
    private BNO055IMU gyro;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    public DcMotor clawSecMotor = null;
    public DcMotor clawMainMotor = null;

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
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    definitieRobot func = new definitieRobot();

    @Override
    public void runOpMode() throws InterruptedException {

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

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        gyro.initialize(imuParameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        func.init(hardwareMap);

        waitForStart();
        runtime.reset();
       // while (opModeIsActive()){
            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            Acceleration gravity = gyro.getGravity();

            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);

            int chosen = 0;

            if(valLeft==0) chosen = 1;
            if(valMid==0) chosen = 2;
            if(valRight==0) chosen = 3;

            switch(chosen)
            {
                case 1:
                    driveFrontStraight(1621, 1621, 1621, 1621, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}
                    clawMainMotor.setPower(0);

                    driveLeftStraight(700, 700, 700, 700, 0, 0.4);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    driveFrontStraight(1883, 1883, 1883, 1883, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    runtime.reset();
                    clawMainMotor.setPower(-0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}

                    driveBackStraight(1943, 1943, 1943, 1943, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    driveLeftStraight(3823, 3823, 3823, 3823, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}


                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}
                    clawMainMotor.setPower(0);

                    driveRightStraight(6313, 6313, 6313, 6313, 0, 0.5);
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}
                    resetDrives();

                    driveFrontStraight(2143, 2143, 2143, 2143, 0, 0.5);
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}
                    resetDrives();


                    clawMainMotor.setPower(-0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}

                    driveBackStraight(2143, 2143, 2143, 2143, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    driveLeftStraight(5863, 5863, 5863, 5863, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}
                    clawMainMotor.setPower(0);

                    clawSecMotor.setPower(-0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.2){}
                    clawSecMotor.setPower(0);

                    driveRightStraight(2000, 2000, 2000, 2000, 0, 0.4);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    driveFrontStraight(1023, 1023, 1023, 1023, 0, 0.4);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}
                    break;

                case 2:
                    driveFrontStraight(1397, 1591, 1570, 1621, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}
                    clawMainMotor.setPower(0);

                    driveRightStraight(400, 400, 400, 400, 0, 0.4);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    driveFrontStraight(1589, 1885, 1874, 1883, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    runtime.reset();
                    clawMainMotor.setPower(-0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}

                    driveBackStraight(1900, 1900, 1900, 1900, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    driveLeftStraight(4523, 4747, 4346, 4302, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}


                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}
                    clawMainMotor.setPower(0);

                    driveRightStraight(6813, 6813, 6813, 6813, 0, 0.5);
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}
                    resetDrives();

                    driveFrontStraight(1900, 1900, 1900, 1900, 0, 0.5);
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}
                    resetDrives();


                    clawMainMotor.setPower(-0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}

                    driveBackStraight(2012, 2012, 2012, 2012, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    driveLeftStraight(6500, 6500, 6500, 6500, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}
                    clawMainMotor.setPower(0);

                    clawSecMotor.setPower(-0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.2){}
                    clawSecMotor.setPower(0);

                    driveRightStraight(1686, 1686, 1686, 1686, 0, 0.4);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    driveFrontStraight(1023, 1023, 1023, 1023, 0, 0.4);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}
                    break;

                case 3:
                    driveFrontStraight(1397, 1591, 1570, 1621, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}
                    clawMainMotor.setPower(0);

                    driveRightStraight(1100, 1100, 1100, 1100, 0, 0.4);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    driveFrontStraight(1883, 1883, 1883, 1883, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    runtime.reset();
                    clawMainMotor.setPower(-0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}

                    driveBackStraight(1900, 1900, 1900, 1900, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    driveLeftStraight(5023, 5047, 5046, 5002, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}


                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}
                    clawMainMotor.setPower(0);

                    driveRightStraight(3903, 3903, 3903, 3903, 0, 0.5);
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}
                    resetDrives();

                    driveFrontStraight(1900, 1900, 1900, 1900, 0, 0.5);
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}
                    resetDrives();


                    clawMainMotor.setPower(-0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}

                    driveBackStraight(2012, 2012, 2012, 2012, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    driveLeftStraight(3800, 3800, 3800, 3800, 0, 0.5);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.5){}
                    clawMainMotor.setPower(0);

                    clawSecMotor.setPower(-0.4);
                    runtime.reset();
                    while(runtime.seconds()<0.2){}
                    clawSecMotor.setPower(0);

                    driveRightStraight(2100, 2100, 2100, 2100, 0, 0.4);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    driveFrontStraight(1023, 1023, 1023, 1023, 0, 0.4);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}
                    break;
            }
        //}
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
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