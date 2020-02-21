package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
@Autonomous(name= "DetectorRosu_Gyro", group="Autonom Cuburi")
//@Disabled//comment out this line before using
public class DetectorRosu_Gyro extends LinearOpMode {
    private BNO055IMU gyro;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    public DcMotor clawSecMotor = null;
    public DcMotor clawMainMotor = null;
    private DcMotor motorLift1 = null;
    private DcMotor motorLift2 = null;

    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = 0;
    private static int valLeft = 0;
    private static int valRight = 0;

    private static float rectHeight = 1.5f/8f;
    private static float rectWidth = .6f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4.5f/8f+offsetX, 3.4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {4.5f/8f+offsetX, 1.4f/8f+offsetY};
    private static float[] rightPos = {4.5f/8f+offsetX, 5.2f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    double powerFL, powerFR, powerBL, powerBR;
    double u =0;

    OpenCvCamera phoneCam;

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        //func.init(hardwareMap);

        waitForStart();
        runtime.reset();
        if(opModeIsActive()){
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
                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }
                    clawMainMotor.setPower(0);

                    driveFrontStraight_u(3000, 3000, 3000, 3000, 32, 0.9, 1);
                    resetDrives();
                    runtime.reset();



                    driveFrontStraight(1000, 400, 1000, 1000, 0, 0.5, 1);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    runtime.reset();
                    clawMainMotor.setPower(-1);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }

                    driveBackStraight(1500, 1300, 1500, 1500, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();


                    RotateRight(0, 0.8, -90);
                    resetDrives();

                    driveFrontStraight(4900, 4500, 4900, 4900, -85, 1, 1);
                    resetDrives();
                    runtime.reset();


                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }
                    clawMainMotor.setPower(0);

                    driveBackStraight(6500, 6000, 6500, 6500, -85, 1, 1);
                    resetDrives();
                    runtime.reset();

                    RotateLeft(-90, 0.8, 0);
                    resetDrives();

                    driveLeftStraight(2000, 1600, 2000, 2000, -10, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    driveFrontStraight(1900, 1500, 1900, 1900, -7, 0.5, 1);
                    resetDrives();
                    runtime.reset();

                    runtime.reset();
                    clawMainMotor.setPower(-1);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }

                    driveBackStraight(1900, 1500, 1900, 1900, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    driveRightStraight(1900, 500, 1900, 1900, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    RotateRight(0, 0.8, -90);
                    resetDrives();

                    driveFrontStraight(7000, 6500, 7000, 7000, -85, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }
                    clawMainMotor.setPower(0);

                    driveBackStraight(4000, 3300, 4000, 4000, -86, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    RotateLeft(-90, 0.8, 0);
                    resetDrives();

                    driveFrontStraight(1600, 1000, 1600, 1600, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    driveFrontStraight(500, 500, 500, 500, 0, 0.5, 1);
                    resetDrives();
                    runtime.reset();

                    runtime.reset();
                    clawMainMotor.setPower(-1);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }

                    driveBackStraight(2000, 1300, 2000, 2000, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();


                    RotateRight(0, 0.8, -90);
                    resetDrives();

                    driveFrontStraight(4900, 3300, 4900, 4900, -85, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }
                    clawMainMotor.setPower(0);

                    driveBackStraight(2000, 1250, 2000, 2000, -87, 0.9, 1);
                    resetDrives();
                    runtime.reset();
                    break;

                case 2:
                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }
                    clawMainMotor.setPower(0);

                    driveFrontStraight(2500, 2500, 2500, 2500, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    driveRightStraight(300, 300, 300, 300, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    driveFrontStraight(550, 550, 550, 550, 0, 0.5, 1);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    runtime.reset();
                    clawMainMotor.setPower(-1);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }

                    driveBackStraight(1100, 1000, 1100, 1100, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    RotateRight(0, 0.8, -90);
                    resetDrives();


                    driveFrontStraight(4800, 4300, 4800, 4800, -84, 0.9, 1);
                    resetDrives();
                    runtime.reset();


                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }
                    clawMainMotor.setPower(0);

                    driveBackStraight(6900, 6300, 6900, 6900, -84, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    RotateLeft(-90, 0.8, 0);
                    resetDrives();

                    driveFrontStraight(900, 900, 900, 900, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    driveFrontStraight(500, 500, 500, 500, 0, 0.5, 1);
                    resetDrives();
                    runtime.reset();

                    runtime.reset();
                    clawMainMotor.setPower(-1);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }

                    driveBackStraight(1000, 1100, 1000, 1000, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();


                    RotateRight(0, 0.8, -90);
                    resetDrives();

                    driveFrontStraight(6900, 6300, 6900, 6900, -84, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }
                    clawMainMotor.setPower(0);

                    driveBackStraight(4400, 3600, 4400, 4400, -84, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    RotateLeft(-90, 0.8, 0);
                    resetDrives();

                    driveFrontStraight(1100, 1100, 1100, 1100, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    driveFrontStraight(700, 700, 700, 700, 0, 0.5, 1);
                    resetDrives();
                    runtime.reset();

                    runtime.reset();
                    clawMainMotor.setPower(-1);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }

                    driveBackStraight(3000, 1300, 3000, 3000, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();


                    RotateRight(0, 0.8, -90);
                    resetDrives();

                    driveFrontStraight(3900, 3600, 3900, 3900, -87, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }
                    clawMainMotor.setPower(0);

                    driveBackStraight(2200, 1200, 2200, 2200, -84, 0.9, 1);
                    resetDrives();
                    runtime.reset();
                    break;
                case 3:
                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }
                    clawMainMotor.setPower(0);

                    driveFrontStraight(2500, 2500, 2500, 2500, -65, 0.9, 1);
                    resetDrives();
                    runtime.reset();



                    driveFrontStraight(850, 550, 850, 850, 0, 0.5, 1);
                    resetDrives();
                    runtime.reset();
                    //while(runtime.seconds()<0.5){}

                    runtime.reset();
                    clawMainMotor.setPower(-1);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }

                    driveBackStraight(1100,750, 1100, 1100, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();


                    RotateRight(0, 0.8, -90);
                    resetDrives();

                    driveFrontStraight(3900, 3500, 3900, 3900, -85, 0.9, 1);
                    resetDrives();
                    runtime.reset();


                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }
                    clawMainMotor.setPower(0);

                    driveBackStraight(6000, 5800, 6000, 6000, -85, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    RotateLeft(-90, 0.8, 0);
                    resetDrives();

                    driveFrontStraight(900, 700, 900, 900, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    driveFrontStraight(900, 700, 900, 900, 0, 0.5, 1);
                    resetDrives();
                    runtime.reset();

                    runtime.reset();
                    clawMainMotor.setPower(-1);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }

                    driveBackStraight(1500, 900, 1500, 1500, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();


                    RotateRight(0, 0.8, -90);
                    resetDrives();

                    driveFrontStraight(6000, 5600, 6000, 6000, -85, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }
                    clawMainMotor.setPower(0);

                    driveBackStraight(5000, 4000, 5000, 5000, -85, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    RotateLeft(-90, 0.8, 0);
                    resetDrives();

                    driveFrontStraight(900, 700, 900, 900, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    driveFrontStraight(900, 700, 900, 900, 0, 0.5, 1);
                    resetDrives();
                    runtime.reset();

                    runtime.reset();
                    clawMainMotor.setPower(-1);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }

                    driveBackStraight(2000, 1100, 2000, 2000, 0, 0.9, 1);
                    resetDrives();
                    runtime.reset();


                    RotateRight(0, 0.8, -90);
                    resetDrives();

                    driveFrontStraight(4500, 4200, 4500, 4500, -85, 0.9, 1);
                    resetDrives();
                    runtime.reset();

                    clawMainMotor.setPower(0.4);
                    runtime.reset();
                    while (runtime.seconds() < 0.5) {
                    }
                    clawMainMotor.setPower(0);


                    //RotateRight(-83, 0.8, -103);
                    //resetDrives();

                    driveBackStraight(1700, 1400, 1700, 1700, -85, 0.9, 1);
                    resetDrives();
                    runtime.reset();
            }


        }
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

    public void driveFrontStraight_u(int t1, int t2, int t3, int t4, double straightAngle, double speed, int P)
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

        while(Math.abs(frontLeft.getCurrentPosition()) < t1-200)
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

        while(Math.abs(frontRight.getCurrentPosition()) < t2-200)
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

        while(Math.abs(frontRight.getCurrentPosition()) < t2-200)
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

        while(Math.abs(frontRight.getCurrentPosition()) < t2-200)
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

        while(Math.abs(frontRight.getCurrentPosition()) < t2-200)
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

        while(Math.abs(frontRight.getCurrentPosition()) < t2-200)
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

        while(Math.abs(frontRight.getCurrentPosition()) < t2-200)
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