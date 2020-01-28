package org.firstinspires.ftc.teamcode.prototypes_and_tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;

@TeleOp(name = "ServoMovement", group = "Concept")
@Disabled
public class ServoMovememt extends OpMode {
    Servo servo =null;
    Servo cunServo = null;
    Controller controller = new Controller();

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        servo = hardwareMap.get (Servo.class, "servo");
        cunServo = hardwareMap.get(Servo.class, "cservo");
        servo.setDirection(Servo.Direction.FORWARD);
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void loop() {
        controller.setValues(gamepad1);

        cunServo.setPosition(controller.rightStickY.getValue()*0.5 + 0.5);

        if (controller.dpadDown.onClick()){
            servo.setPosition(servo.getPosition()-0.1) ;
        }
        else if (controller.dpadUp.onClick()) {
            servo.setPosition(servo.getPosition()+0.1) ;
        }

        telemetry.addData("continous servo power", cunServo.getPosition());
        telemetry.addData("servo" , servo.getPosition());

        controller.setPrevValues();

    }

}