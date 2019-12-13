package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@Autonomous(name= "DetectorAlbastru", group="Sky autonomous")
//@Disabled//comment out this line before using
public class DetectorAlbastru extends LinearOpMode {
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

            if(valLeft==0) chosen = 1;
            if(valMid==0) chosen = 2;
            if(valRight==0) chosen = 3;

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
                    func.move_front(0.5, 1434, 1498, 1690, 1545);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.15) {
                        func.clawMainMotor.setPower(0.5);
                    }
                    sleep(250);

                    func.move_left(0.5, 453, 422, 410, 376);
                    WAIT();
                    func.resetDrives();

                    func.move_front(0.5, 1340, 1554, 1565, 1602);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<1) {
                        func.clawMainMotor.setPower(-0.5);
                    }
                    func.resetDrives();

                    func.move_back(0.5, 896, 1024, 942, 1032);
                    WAIT();
                    func.resetDrives();

                    func.move_left(0.5, 3531, 3737, 4038, 3861);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.15) {
                        func.clawMainMotor.setPower(0.5);
                    }
                    func.clawMainMotor.setPower(0.0);

                    func.move_right(0.6, 4662, 5358, 5793, 5383);
                    WAIT();
                    func.resetDrives();

                    sleep(250);

                    func.move_front(0.7, 2117, 2392, 2395, 2346);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<1) {
                        func.clawMainMotor.setPower(-0.5);
                    }

                    func.move_back(0.4, 1520, 1572, 1694, 1681);
                    WAIT();
                    func.resetDrives();

                    func.move_left(0.5, 5908, 6280, 6734, 6392);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.15) {
                        func.clawMainMotor.setPower(0.5);
                    }
                    func.clawMainMotor.setPower(0.0);

                    func.move_right(0.5, 560, 550, 600, 567);
                    WAIT();
                    func.resetDrives();

                    func.move_front(0.5, 1500, 1500, 1500, 1500);
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

                    func.move_right(0.5, 288, 359, 291, 298);
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

                    func.move_left(0.5, 5075, 5195, 5617, 4626);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.15) {
                        func.clawMainMotor.setPower(0.5);
                    }

                    func.move_right(0.7, 5698, 6542, 5681, 6545);
                    WAIT();
                    func.resetDrives();

                    func.move_front(0.7, 1788, 2226, 2222, 2150);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<1) {
                        func.clawMainMotor.setPower(-0.5);
                    }

                    func.move_back(0.7, 1788, 2226, 2222, 2150);
                    WAIT();
                    func.resetDrives();

                    func.move_left(0.7, 7000, 7000, 7000, 7000);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.15) {
                        func.clawMainMotor.setPower(0.5);
                    }

                    func.move_right(0.7, 800, 800, 800, 800);
                    WAIT();
                    func.resetDrives();

                    func.move_front(0.7, 1700, 1700, 1700, 1700);
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

                    func.move_right(0.5, 1041, 1246, 1245, 1184);
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

                    func.move_left(0.5, 5753, 6879, 7555, 6530);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.15) {
                        func.clawMainMotor.setPower(0.5);
                    }
                    sleep(500);
                    func.clawMainMotor.setPower(0.0);

                    func.move_right(0.7, 3000, 3000, 3000, 3000);
                    WAIT();
                    func.resetDrives();

                    func.move_front(0.7, 2500, 2500, 2500, 2500);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<0.25) {
                        func.clawMainMotor.setPower(-0.5);
                    }
                    sleep(500);

                    func.move_back(0.5, 2000, 2000, 2000, 2000);
                    WAIT();
                    func.resetDrives();


                    func.move_left(0.5, 3539, 3593, 3825, 3661);
                    WAIT();
                    func.resetDrives();

                    func.rotate_left(0.5, 500, 500, 500, 500);
                    WAIT();
                    func.resetDrives();

                    runtime.reset();
                    while(runtime.seconds()<1) {
                        func.clawMainMotor.setPower(1);
                    }
                    sleep(500);
                    func.clawMainMotor.setPower(0.0);

                    func.move_right(0.7, 1185, 1242, 1316, 1273);
                    WAIT();
                    func.resetDrives();

                    func.move_front(0.7, 1000, 1000, 1000, 1000);
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