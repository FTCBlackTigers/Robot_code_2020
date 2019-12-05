package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.robot_systems.MecanumDrive;


@TeleOp(name = "mecanum drive", group = "teleop")
public class MecanumTeleop extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Controller driver = new Controller();
    Controller operator = new Controller();
    MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void init() {
        mecanumDrive.init(hardwareMap, this);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        driver.setValues(gamepad1);
        operator.setValues(gamepad2);
        mecanumDrive.teleopMotion(driver , operator);
        driver.setPrevValues();
        operator.setPrevValues();
    }
}
