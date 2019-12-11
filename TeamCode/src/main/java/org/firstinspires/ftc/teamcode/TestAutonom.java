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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;




@Autonomous(name="Basic: Linear OpMode", group="Linear Opmode")

public class TestAutonom extends LinearOpMode {
   // private HardwareRobot func = null;


    private ElapsedTime runtime = new ElapsedTime();
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

        waitForStart();
        runtime.reset();
        func.move_left(0.5,2981, 2967, 2940, 2890);
        WAIT();
        func.resetDrives();

        sleep(1000);
        func.clawSecMotor.setPower(0.4);

        func.move_right(0.5, 2781, 2767, 2740, 2690);
        WAIT();
        func.resetDrives();

        func.move_back(0.5,1346, 1504, 2316, 1865);
        WAIT();
        func.resetDrives();

        sleep(1000);

        func.clawSecMotor.setPower(0.0);

        sleep(1000);

        func.rotate_right(0.5,162, 133, 180, 164);
        WAIT();
        func.resetDrives();

        sleep(1000);

        func.move_back(0.5,2282, 2394, 2680, 2639);
        WAIT();
        func.resetDrives();

    }
    public void WAIT() {
        while (func.isBusy()) {

        }
    }

}
