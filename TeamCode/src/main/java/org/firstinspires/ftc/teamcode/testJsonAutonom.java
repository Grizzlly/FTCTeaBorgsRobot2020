package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileNotFoundException;


@Autonomous(name="testJsonAutonom", group="Linear Opmode")
//@Disabled
public class testJsonAutonom extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareRecorder recorder = new HardwareRecorder("");
    private int pozitie;
    static final int COUNTS = 1112;

    public definitieRobot robot = new definitieRobot();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        //robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        recorder.InitMotors(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);
        waitForStart();
        while(opModeIsActive())
        {
            try {
                //recorder.Play();
                //telemetry.addLine("playing...");
                //telemetry.update();
            }catch (Exception ex)
            {
                telemetry.addLine(ex.getMessage());
                telemetry.update();
            }
            runtime.reset();
            while(runtime.seconds()<30){telemetry.addLine("playing...");telemetry.update();}

        }
    }

    public void WAIT() {
        while (!(robot.frontLeft.getTargetPosition()-100 <= robot.frontLeft.getCurrentPosition() && robot.frontLeft.getCurrentPosition() <= robot.frontLeft.getTargetPosition()+100)) {
            /*telemetry.addData("", robot.frontLeft.getCurrentPosition()/1112*3.14*10);
            telemetry.addData("", robot.frontRight.getCurrentPosition());
            telemetry.addData("", robot.backLeft.getCurrentPosition());
            telemetry.addData("", robot.backRight.getCurrentPosition());
            telemetry.addData("", robot.DrivingTarget);
            telemetry.update();*/
        }
    }

}