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

package org.firstinspires.ftc.teamcode.prototypes_and_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.controller.Controller;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "intake", group = "Concept")
public class ProtIntake extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorIntake = null;
    private DcMotor motorRamp = null;
    private double power = 0.5;
    Controller controller = new Controller();



    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        motorRamp = hardwareMap.get(DcMotor.class, "MotorRamp");
        motorIntake = hardwareMap.get(DcMotor.class, "MotorIntake");
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {
    }

    boolean dUp;
    boolean dUpPrev;
    boolean dDown;
    boolean dDownPrev;

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        controller.setValues(gamepad1);
        motorRamp.setPower(controller.leftStickY.getValue());

        if (controller.leftStickY.getValue() == 0){
            motorRamp.setPower(0);
        }


        if (controller.leftTrigger.getValue() != 0) {
            motorIntake.setPower(-power);
            if (dUp&&!dUpPrev) {
                power = power + 0.1;
                power = Range.clip(power, 0.1, 1);
            }
            else if (dDown&&!dDownPrev) {
                power = power - 0.1;
                power = Range.clip(power, 0.1, 1);
            }
        }
        else{
            motorIntake.setPower(0);
        }
        if(controller.b.onClick()){
            motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        controller.setPrevValues();
        telemetry.addData("Encoder", motorRamp.getCurrentPosition());
    }


}

