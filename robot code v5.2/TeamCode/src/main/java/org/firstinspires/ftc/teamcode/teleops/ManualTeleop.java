package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.robot_systems.Robot;

@TeleOp(name = "ManualTeleOp", group = "TeleOp")
public class ManualTeleop extends OpMode {
    Robot robot =new Robot();
    Controller driver = new Controller();
    Controller operator= new Controller();
    @Override
    public void init() {
        robot.init(hardwareMap, this);
    }

    @Override
    public void loop() {
        driver.setValues(gamepad1);
        operator.setValues(gamepad2);
        robot.intake.manualTeleop(driver, operator );
        robot.movingStoneArm.manualTeleop(driver, operator);
        driver.setPrevValues();
        operator.setPrevValues();
    }
}