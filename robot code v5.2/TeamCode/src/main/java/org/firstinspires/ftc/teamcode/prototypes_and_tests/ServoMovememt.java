package org.firstinspires.ftc.teamcode.prototypes_and_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;

@TeleOp(name = "ServoMovement", group = "Concept")
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
        servo = hardwareMap.get (Servo.class, "servo");
        servo.setDirection(Servo.Direction.FORWARD);

       //servo=hardwareMap.get( "servo", )
    }


    @Override
    public void init_loop() {



    }


    @Override
    public void loop() {
        telemetry.addData("servo" , servo.getPosition());
        controller.setValues(gamepad1);
        if (controller.dpadDown.onClick()){
         servo.setPosition(servo.getPosition()-0.1) ;
        }
        else if (controller.dpadUp.onClick()) {
            servo.setPosition(servo.getPosition()+0.1) ;
        }
        controller.setPrevValues();
    }

}