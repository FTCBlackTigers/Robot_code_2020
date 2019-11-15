package org.firstinspires.ftc.teamcode.prototypes_and_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.robot_systems.FoundationMove;
import org.firstinspires.ftc.teamcode.robot_systems.Intake;
import org.firstinspires.ftc.teamcode.robot_systems.MecanumDrive;

@TeleOp(name = "TestSystem", group = "teleop")
public class TestSystem extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    FoundationMove foundation= new FoundationMove();
    private ElapsedTime runtime = new ElapsedTime();
    Controller driver = new Controller();
    Controller oparetor = new Controller();
    @Override
    public void init() {
        mecanumDrive.init(hardwareMap, this);
        foundation.init(hardwareMap, this);
    }

    @Override
    public void loop() {
        driver.setValues(gamepad1);
        oparetor.setValues(gamepad2);
        mecanumDrive.teleopMotion(driver,oparetor);
        foundation.teleopMotion(driver, oparetor);
        driver.setPrevValues();
        oparetor.setPrevValues();



    }
}
