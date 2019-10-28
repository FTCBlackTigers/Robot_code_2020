package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.Controller;


@TeleOp(name = "mecanum drive", group = "teleop")
public class MecanumTeleop extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Controller driver = new Controller();
    Controller oparetor = new Controller();
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
        oparetor.setValues(gamepad2);
        mecanumDrive.teleopMotion(driver , oparetor);
        driver.setPrevValues();
        oparetor.setPrevValues();
    }
}
