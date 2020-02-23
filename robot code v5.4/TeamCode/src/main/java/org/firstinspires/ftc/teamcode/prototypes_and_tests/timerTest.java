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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;


@TeleOp(name = "timerTest", group = "Concept")
@Disabled
public class timerTest extends OpMode {
  public void rotateLeft(int time){
    servo.setPosition(1);
    rotateStop(time);
  }
  public void rotateRight(int time){
    servo.setPosition(0);
    rotateStop(time);
  }
  public void  rotateStop(int time){
    t.schedule(new TimerTask() {
      @Override
      public void run() {
        telemetry.addLine("calling from the delay").addData("time after", getRuntime());
        servo.setPosition(0.5);
      }
    }, time);
  }

  private ElapsedTime runtime = new ElapsedTime();
  final Timer t = new java.util.Timer();
  Servo servo;

  @Override
  public void init() {
    servo = hardwareMap.get(Servo.class, "servo");
    telemetry.addData("Status", "Initialized");
    telemetry.addData("time start", getRuntime());
    t.schedule(new TimerTask() {
      @Override
      public void run() {
        telemetry.addLine("calling from the delay").addData("time after", getRuntime());
        //t.cancel();
      }
    }, 2000);
  }

  @Override
  public void init_loop() {
  }


  @Override
  public void start() {
    runtime.reset();
  }

  
  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());

    if(gamepad1.a){
      telemetry.addData("a is pressed", getRuntime());
     rotateLeft(750);
    }
    if(gamepad1.b){
      rotateRight(750);
    }
    telemetry.addData( "servo position",servo.getPosition());
  }
}