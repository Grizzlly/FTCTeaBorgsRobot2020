package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="autonom3", group="Linear Opmode")
@Disabled
public class autonom3 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private int pozitie;
    private final int COUNTS = 1120;

    public definitieRobot func = new definitieRobot();

    HardwareMap hwMap;

    public autonom3(HardwareMap hw)
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