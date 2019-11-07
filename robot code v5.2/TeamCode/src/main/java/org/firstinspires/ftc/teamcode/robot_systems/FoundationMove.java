package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;

public class FoundationMove extends SubSystem {
    Servo foundServo;
    double upPos = 0.6;
    double downPos = 0;

    @Override
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        foundServo = hardwareMap.get(Servo.class, "foundServo");
        up();
    }

    @Override
    public void teleopMotion(Controller driver, Controller operator){
        if(driver.y.onClick()){
            up();
        }
        if(driver.x.onClick()) {
            down();
        }
    }
    public void up() {
        foundServo.setPosition(upPos);
    }
    public void down() {
        foundServo.setPosition(downPos);
    }
}
