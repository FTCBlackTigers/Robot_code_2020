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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "intake", group = "Concept")
public class propintake extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor = null;
    private Servo servoLeft = null;
    private Servo servoRight = null;
    private double power = 0.5;
    private double upPos = 0.9;
    private double downPos = 0.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        motor = hardwareMap.get(DcMotor.class, "Motor");
        servoLeft = hardwareMap.get(Servo.class, "ServoLeft");
        servoRight = hardwareMap.get(Servo.class, "ServoRight");
        servoLeft.setDirection(Servo.Direction.FORWARD);
        servoRight.setDirection(Servo.Direction.REVERSE);
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    boolean dUpPrev;
    boolean dDownPrev;
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        boolean dUp=gamepad1.dpad_up;
        boolean dDown=gamepad1.dpad_down;
        if (gamepad1.right_trigger != 0) {
            motor.setPower(power);
            if (dUp&&!dUpPrev) {
                    power = power + 0.1;
                    power = Range.clip(power, 0.1, 1);
                } else if (dDown&&!dDownPrev) {
                    power = power - 0.1;
                    power = Range.clip(power, 0.1, 1);
            }

        }
        if (gamepad1.left_trigger != 0) {
            motor.setPower(-power);
            if (dUp&&!dUpPrev) {
                power = power + 0.1;
                power = Range.clip(power, 0.1, 1);
            }
            else if (dDown&&!dDownPrev) {
                power = power - 0.1;
                power = Range.clip(power, 0.1, 1);
            }
        }
        else {
            motor.setPower(0);
        }
        if (gamepad1.y) {
            servoRight.setPosition(upPos);
            servoLeft.setPosition(upPos);
            if (dUp&&!dUpPrev) {
                upPos = upPos + 0.1;
                upPos = Range.clip(upPos, 0.1, 1);
            } else if (dDown&&!dDownPrev) {
                upPos = upPos - 0.1;
                upPos = Range.clip(upPos, 0.1, 1);

            }
        } else if (gamepad1.a) {
            servoRight.setPosition(downPos);
            servoLeft.setPosition(downPos);
        }
        telemetry.addData("power", power);
        telemetry.addData("position", upPos);

        dUpPrev = dUp;
        dDownPrev = dDown;
    }
}

