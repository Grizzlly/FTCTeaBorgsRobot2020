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

    public definitieRobot func = new definitieRobot();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        func.init(hardwareMap);

        func.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        func.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        func.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        func.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if(opModeIsActive()) {
            func.init(hardwareMap);

            func.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            func.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            func.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            func.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            waitForStart();
            runtime.reset();
            func.move_left(0.3, 2901, 2816, 2808, 2833);
            WAIT();
            func.resetDrives();
            func.clawSecMotor.setPower(-0.4);
            sleep(1500);

            func.move_right(0.5, 3181, 3565, 4153, 3529);
            WAIT();
            func.resetDrives();

            sleep(1000);

            while(runtime.seconds() < 0.5);
            func.clawSecMotor.setPower(0.4);

            sleep(1000);

            func.clawSecMotor.setPower(0.0);

            sleep(1000);

            func.rotate_left(0.5, 354, 455, 385, 403);
            WAIT();
            func.resetDrives();

            func.move_back(0.5, 2347, 2427, 2555, 2500);
            WAIT();
            func.resetDrives();

            func.move_left(0.5, 1635, 1688, 1704, 1658);
            WAIT();
            func.resetDrives();

            func.rotate_right(0.3, 204, 279, 203, 204);
            WAIT();
            func.resetDrives();

            func.move_front(0.5, 962, 1021, 1103, 1140);
            WAIT();
            func.resetDrives();

            func.rotate_left(0.3, 273, 167, 335, 228);
            WAIT();
            func.resetDrives();

            func.move_right(0.5, 1250, 1406, 1492, 1382);
            WAIT();
            func.resetDrives();

            func.move_back(0.5, 1870, 2074, 2149, 2037);
            WAIT();
            func.resetDrives();
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