package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="autonom1", group="Linear Opmode")
//@Disabled
public class autonom1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private int pozitie;
    static final int COUNTS = 1112;

    public definitieRobot robot = new definitieRobot();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if(opModeIsActive()) {
            /*robot.move_front(1.0,860);
            WAIT(true);
            robot.rotate_left(0.5,800);
            WAIT(true);
            robot.move_front(0.5,650);
            WAIT(true);
            robot.move_left(0.5,2900);
            WAIT(true);
            robot.rotate_left(0.5, 1330);
            WAIT(true);
            //WAIT(true);
            ///waiting de o sec
            robot.move_back(1.0,3300);
            WAIT(true);

            robot.rotate_left(1.0,1420);
            WAIT(true);
            robot.move_back(1.0,3000);
            WAIT(true);
            */
        }
    }

    public void WAIT(boolean busy){
        if(busy == true)
            while (opModeIsActive() && robot.isBusy()) {

            }

        /*else
            while(opModeIsActive() && !robot.positionReached()){

            }*/

        robot.resetDrives();
    }

}