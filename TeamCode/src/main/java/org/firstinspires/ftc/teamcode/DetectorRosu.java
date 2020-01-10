package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
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
@Autonomous(name= "DetectorRosu", group="Sky autonomous")
@Disabled//comment out this line before using
public class DetectorRosu extends LinearOpMode {
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

            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);

            int chosen = 0;

            if(valLeft==0) chosen = 3;
            if(valMid==0) chosen = 2;
            if(valRight==0) chosen = 1;

            switch(chosen)
            {
                case 1:
                   // autonom2 au2 = new autonom2(hardwareMap);
                   // au2.runOpMode();

                    func.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    func.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    func.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    func.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    runtime.reset();

                    func.move_front(0.5, 1209, 1309, 1309, 1385);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.15) {
                        func.clawMainMotor.setPower(0.5);
                    }
                    sleep(250);

                    func.move_right(0.5, 841, 1046, 1045, 984);
                    WAIT();
                    func.resetDrives();

                    func.move_front(0.5, 1391, 1552, 1562, 1533);
                    WAIT();
                    func.resetDrives();

                    while(runtime.seconds()<3) {
                        func.clawMainMotor.setPower(-0.5);
                    }

                    func.move_back(0.5, 933, 1013, 1142, 1018);
                    WAIT();
                    func.resetDrives();

                    func.move_right(0.5, 2153, 3279, 3955, 2930);
                    WAIT();
                    func.resetDrives();

                    func.rotate_right(0.5, 399, 416, 445, 449);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.15) {
                        func.clawMainMotor.setPower(0.5);
                    }
                    sleep(500);
                    func.clawMainMotor.setPower(0.0);

                    func.move_left(0.7, 5300, 5200, 5300, 5050);
                    WAIT();
                    func.resetDrives();

                    func.rotate_right(0.5, 222, 290, 229, 234);
                    WAIT();
                    func.resetDrives();

                    func.move_front(0.5, 1200, 1386, 1387, 1437);
                    WAIT();
                    func.resetDrives();


                    runtime.reset();
                    while(runtime.seconds()<0.25) {
                        func.clawMainMotor.setPower(-0.5);
                    }
                    sleep(500);

                    func.move_back(0.5, 1333, 1412, 1447, 1422);
                    WAIT();
                    func.resetDrives();

                    func.move_right(0.7, 3228, 3405, 3801, 3965);
                    WAIT();
                    func.resetDrives();

                    func.rotate_right(0.5, 164, 193, 238, 264);
                    WAIT();
                    func.resetDrives();
                    //
                    func.move_right(0.7, 1708, 1867, 2182, 2176);
                    WAIT();
                    func.resetDrives();

                    func.rotate_right(0.5, 301, 310, 343, 351);
                    WAIT();
                    func.resetDrives();

                    func.move_right(0.5, 381, 404, 488, 457);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.5) {
                        func.clawMainMotor.setPower(-1);
                    }
                    sleep(500);
                    func.clawMainMotor.setPower(0.0);

                    //func.rotate_right(0.5, 137, 151, 166, 176);
                    //WAIT();
                    //func.resetDrives();

                    func.move_left(0.5, 946, 957, 988, 969);
                    WAIT();
                    func.resetDrives();

                    func.rotate_right(0.5, 137, 151, 166, 176);
                    WAIT();
                    func.resetDrives();

                    func.move_front(0.7, 1120, 1120, 1120, 1120);
                    WAIT();
                    func.resetDrives();
                    break;

                case 2:
                    //autonom3 au3 = new autonom3(hardwareMap);
                    //au3.GO();

                    func.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    func.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    func.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    func.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    runtime.reset();

                    func.move_front(0.5, 1765, 1853, 1893, 1961);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.15) {
                        func.clawMainMotor.setPower(0.5);
                    }

                    func.rotate_right(0.5, 88, 159, 91, 98);
                    WAIT();
                    func.resetDrives();

                    func.move_front(0.5, 996, 1066, 1066, 1092);
                    WAIT();
                    func.resetDrives();

                    while(runtime.seconds()<2) {
                        func.clawMainMotor.setPower(-0.5);
                    }

                    func.move_back(0.5, 1198, 1328, 1376, 1330);
                    WAIT();
                    func.resetDrives();

                    func.move_right(0.5, 3575, 3695, 4117, 4126);
                    WAIT();
                    func.resetDrives();

                    func.rotate_right(0.5, 394, 357, 517, 425);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.15) {
                        func.clawMainMotor.setPower(0.5);
                    }

                    func.move_left(0.7, 6098, 6942, 6081, 6945);
                    WAIT();
                    func.resetDrives();

                    func.rotate_right(0.5, 144, 198, 154, 179);
                    WAIT();
                    func.resetDrives();

                    func.move_front(0.5, 1497, 1594, 1636, 1665);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.5) {
                        func.clawMainMotor.setPower(-0.5);
                    }

                    func.move_back(0.5, 1274, 1410, 1455, 1394);
                    WAIT();
                    func.resetDrives();

                    func.move_right(0.5, 3300, 3487, 3873, 3743);
                    WAIT();
                    func.resetDrives();

                    func.rotate_right(0.5, 334, 173, 324, 234);
                    WAIT();
                    func.resetDrives();

                    func.move_right(0.5, 2797, 3441, 4086, 3515);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.15) {
                        func.clawMainMotor.setPower(0.5);
                    }

                    func.move_left(0.5, 1188, 1222, 1276, 1234);
                    WAIT();
                    func.resetDrives();

                    //func.rotate_right(0.5, 274, 350, 272, 276);
                    //WAIT();
                    //func.resetDrives();

                    func.move_front(0.5, 1230, 1334, 1357, 1322);
                    WAIT();
                    func.resetDrives();
                    break;

                case 3:
                    //autonom4 au4 = new autonom4(hardwareMap);
                   // au4.runOpMode();

                    func.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    func.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    func.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    func.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    runtime.reset();
                    func.move_front(0.5, 1209, 1309, 1309, 1385);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.15) {
                        func.clawMainMotor.setPower(0.5);
                    }
                    sleep(250);

                    func.move_left(0.5, 441, 646, 645, 584);
                    WAIT();
                    func.resetDrives();

                    func.move_front(0.5, 1391, 1552, 1562, 1533);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<1) {
                        func.clawMainMotor.setPower(-0.5);
                    }

                    func.move_back(0.5, 933, 1013, 1142, 1018);
                    WAIT();
                    func.resetDrives();

                    func.move_right(0.5, 4153, 5279, 5955, 4930);
                    WAIT();
                    func.resetDrives();

                    func.rotate_right(0.5, 399, 416, 445, 449);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.15) {
                        func.clawMainMotor.setPower(0.5);
                    }
                    sleep(500);
                    func.clawMainMotor.setPower(0.0);

                    func.move_left(0.7, 5600, 5600, 5600, 5600);
                    WAIT();
                    func.resetDrives();
                    sleep(500);

                    func.rotate_left(0.5, 1600, 1600, 1600, 1600);
                    WAIT();
                    func.resetDrives();

                    func.move_right(0.5, 1900, 1900, 1900, 1900);
                    WAIT();
                    func.resetDrives();

                    func.rotate_left(0.5, 400, 400, 400, 400);
                    WAIT();
                    func.resetDrives();

                    func.move_front(0.7, 300, 300, 300, 300);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.25) {
                        func.clawMainMotor.setPower(-0.5);
                    }
                    sleep(500);

                    func.move_left(0.5, 1900, 1900, 1900, 1900);
                    WAIT();
                    func.resetDrives();

                    func.rotate_right(0.5, 1900, 1900, 1900, 1900);
                    WAIT();
                    func.resetDrives();

                    func.move_right(0.7, 5600, 5600, 5600, 5600);
                    WAIT();
                    func.resetDrives();
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
    public void WAIT() {
        while (!(func.frontLeft.getTargetPosition()-100 <= func.frontLeft.getCurrentPosition() && func.frontLeft.getCurrentPosition() <= func.frontLeft.getTargetPosition()+100)) {
            /*telemetry.addData("", robot.frontLeft.getCurrentPosition()/1112*3.14*10);
            telemetry.addData("", robot.frontRight.getCurrentPosition());
            telemetry.addData("", robot.backLeft.getCurrentPosition());
            telemetry.addData("", robot.backRight.getCurrentPosition());
            telemetry.addData("", robot.DrivingTarget);
            telemetry.update();*/
        }
    }
}