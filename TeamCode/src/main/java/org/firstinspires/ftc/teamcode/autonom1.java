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
            func.move_left(0.5,2981, 2967, 2940, 2890);
            WAIT();
            func.resetDrives();
            func.clawSecMotor.setPower(-0.4);
            sleep(1500);

            func.move_right(0.5, 2781, 2767, 2740, 2690);
            WAIT();
            func.resetDrives();

            sleep(1000);

            while(runtime.seconds() < 0.5);
                func.clawSecMotor.setPower(0.4);

            sleep(1000);

            func.clawSecMotor.setPower(0.0);

            sleep(1000);

            func.move_back(0.5,1346, 1504, 2316, 1865);
            WAIT();
            func.resetDrives();

            sleep(1000);

            func.rotate_right(0.5,162, 133, 180, 164);
            WAIT();
            func.resetDrives();

            sleep(1000);

            func.move_back(0.5,2282, 2394, 2680, 2639);
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