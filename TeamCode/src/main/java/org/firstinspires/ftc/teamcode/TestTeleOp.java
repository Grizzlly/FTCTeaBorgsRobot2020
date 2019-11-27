/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TestTeleOp", group="Linear Opmode")
//@Disabled
public class TestTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor motorLift = null;
    private DcMotor clawMainMotor = null;
    private DcMotor clawSecMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft  = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRight = hardwareMap.get(DcMotor.class, "fr_motor");
        backLeft  = hardwareMap.get(DcMotor.class, "fl_motor");
        backRight = hardwareMap.get(DcMotor.class, "fr_motor");
        motorLift = hardwareMap.get(DcMotor.class, "motor_lift");
        clawMainMotor  = hardwareMap.get(DcMotor.class, "clawMainMotor");
        clawSecMotor = hardwareMap.get(DcMotor.class, "clawSecMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        clawMainMotor.setDirection(DcMotor.Direction.FORWARD);
        clawSecMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double  omniSurpress = 0.3;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //omni
            double gamepadLeftY = -gamepad2.right_stick_y;
            double gamepadLeftX = -gamepad2.right_stick_x;
            double gamepadRightX = gamepad2.left_stick_x;

            double powerFrontLeft = -gamepadLeftY - gamepadLeftX - gamepadRightX;
            double powerFrontRight = gamepadLeftY - gamepadLeftX - gamepadRightX;
            double powerBackLeft = -gamepadLeftY + gamepadLeftX - gamepadRightX;
            double powerBackRight = gamepadLeftY + gamepadLeftX - gamepadRightX;

            double powerLatchingUp = -gamepad1.right_trigger;
            double powerLatchingDown = gamepad1.left_trigger;

            powerFrontLeft = Range.clip(powerFrontLeft, -1, 1);
            powerFrontRight = Range.clip(powerFrontRight, -1, 1);
            powerBackLeft = Range.clip(powerBackLeft, -1, 1);
            powerBackRight = Range.clip(powerBackRight, -1, 1);
            powerLatchingUp = Range.clip(powerLatchingUp, -1, 1);
            powerLatchingDown = Range.clip(powerLatchingDown, -1, 1);



           /* if (gamepad2.right_bumper == true) {
                powerFrontLeft = powerFrontLeft *   omniSurpress;
                powerBackLeft = powerBackLeft *     omniSurpress;
                powerFrontRight = powerFrontRight * omniSurpress;
                powerBackRight = powerBackRight *   omniSurpress;

            }*/

            frontLeft.setPower(powerFrontLeft);
            frontRight.setPower(powerFrontRight);
            backLeft.setPower(powerBackLeft);
            backRight.setPower(powerBackRight);
            //omni

            //lift
            motorLift.setPower(powerLatchingUp);
            motorLift.setPower(powerLatchingDown);
            //lift

            //claws
            if(gamepad1.dpad_up==true)
            {
                clawMainMotor.setPower(0.5);
            }
            else if(gamepad1.dpad_down==true)
            {
                clawMainMotor.setPower(-0.5);
            }
            else {
                clawMainMotor.setPower(0.0);
            }
            if(gamepad1.dpad_left==true)
            {
                clawSecMotor.setPower(0.5);
            }
            else if(gamepad1.dpad_right==true)
            {
                clawSecMotor.setPower(-0.5);
            }
            else {
                clawSecMotor.setPower(0.0);
            }
            //claws

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front Left Power", ": " + powerFrontLeft);
            telemetry.addData("Front Right Power", ": " + powerFrontRight);
            telemetry.addData("Back Left Power", ": " + powerBackLeft);
            telemetry.addData("Back Right Power", ": " + powerBackRight);

            telemetry.update();
        }
    }
}
