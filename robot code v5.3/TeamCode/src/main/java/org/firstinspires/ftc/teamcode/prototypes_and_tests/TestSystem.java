package org.firstinspires.ftc.teamcode.prototypes_and_tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.robot_systems.Intake;
import org.firstinspires.ftc.teamcode.robot_systems.Intake2;
import org.firstinspires.ftc.teamcode.robot_systems.MovingStoneArm;

@TeleOp(name = "TestSystem", group = "teleop")
//@Disabled
public class TestSystem extends OpMode {
    Intake2 intake = new Intake2();
    private ElapsedTime runtime = new ElapsedTime();
    Controller driver = new Controller();
    Controller oparetor = new Controller();
    @Override
    public void init() {
        intake.init(hardwareMap, this);
    }

    @Override
    public void loop() {
        driver.setValues(gamepad1);
        oparetor.setValues(gamepad2);
        intake.teleopMotion(driver, oparetor);
        driver.setPrevValues();
        oparetor.setPrevValues();
    }
}
