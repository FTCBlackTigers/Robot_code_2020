package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;

public class FoundationMove extends SubSystem {
    Servo foundServo;
    final double UP_POS = 0.6;
    final double DOWN_POS = 0;
    final double GRAB_STONE = 0.5;

    @Override
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
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
        foundServo.setPosition(UP_POS);
    }
    public void down() {
        foundServo.setPosition(DOWN_POS);
    }
    public void grab() { foundServo.setPosition(GRAB_STONE);}
}
