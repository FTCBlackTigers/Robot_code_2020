package org.firstinspires.ftc.teamcode.prototypes_and_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;

@TeleOp(name = "intake", group = "Concept")
public class ServoMovememt extends OpMode {
    Servo servo =null;
    Controller controller = new Controller();

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        servo.setDirection(Servo.Direction.FORWARD);
       //servo=hardwareMap.get( "servo", )
    }


    @Override
    public void init_loop() {



    }


    @Override
    public void loop() {
        controller.setValues(gamepad1);
        if (controller .leftStickY.getValue() <0){
         servo.setPosition(servo.getPosition()-0.1) ;
        }
        else if (controller .leftStickY.getValue()>0) {
            servo.setPosition(servo.getPosition()+0.1) ;
        }

    }

}