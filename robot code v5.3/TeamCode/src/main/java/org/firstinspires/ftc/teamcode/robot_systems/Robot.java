package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controller.Controller;

public class Robot {
    public MecanumDrive mecanumDrive = new MecanumDrive();
    public Intake2 intake = new Intake2();
    public FoundationMove foundationMove = new FoundationMove();
    public Lift lift = new Lift();

    public void init(HardwareMap hardwareMap , OpMode opMode){
        mecanumDrive.init(hardwareMap , opMode);
        intake.init(hardwareMap , opMode);
        foundationMove.init(hardwareMap , opMode);
        lift.init(hardwareMap,opMode);
    }
    public void teleop(Controller driver , Controller operator){
        mecanumDrive.teleopMotion(driver, operator);
        intake.teleopMotion(driver, operator);
        foundationMove.teleopMotion(driver, operator);
        lift.manualTeleop(operator);
    }
}

