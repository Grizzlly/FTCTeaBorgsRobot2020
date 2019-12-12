package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="autonom2", group="Linear Opmode")
//@Disabled
public class autonom2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private int pozitie;
    private final int COUNTS = 1120;

    public definitieRobot func = new definitieRobot();

    HardwareMap hwMap;

    public autonom2(HardwareMap hw)
    {
        hwMap=hw;
    }



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        func.init(hwMap);

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

            func.move_front(0.7, COUNTS, COUNTS, COUNTS, COUNTS);
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