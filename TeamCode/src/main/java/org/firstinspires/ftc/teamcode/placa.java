package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="placa", group="Linear Opmode")
@Disabled
public class placa extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public definitieRobot robot = new definitieRobot();



    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        //robot.tel=telemetry;

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // Wait for the game to start (driver presses PLAY)
        runtime.reset();
        /*robot.move_front(1, 80);
        WAIT();
        robot.resetDrives();
        robot.move_left(1, 100);
        WAIT();
        robot.resetDrives();
        robot.move_right(1, 50);
        WAIT();
        robot.resetDrives();
        robot.move_front(1, 66);
        WAIT();*/
        //WAIT(robot.frontLeft.getTargetPosition());
        //run until the end of the match (driver presses STOP)
    }

    //if !(robot.frontLeft.getTargetPosition()-10 <= robot.frontLeft.getCurrentPosition() && value <= robot.frontLeft.getTargetPosition()+10)

    public void WAIT() {
        while (!(robot.frontLeft.getTargetPosition()-100 <= robot.frontLeft.getCurrentPosition() && robot.frontLeft.getCurrentPosition() <= robot.frontLeft.getTargetPosition()+100)) {
            telemetry.addData("", robot.frontLeft.getCurrentPosition()/1112*3.14*10);
            telemetry.addData("", robot.frontRight.getCurrentPosition());
            telemetry.addData("", robot.backLeft.getCurrentPosition());
            telemetry.addData("", robot.backRight.getCurrentPosition());
            telemetry.addData("", robot.DrivingTarget);
            telemetry.update();
        }
    }

    //public void stop()
    //{}
}
