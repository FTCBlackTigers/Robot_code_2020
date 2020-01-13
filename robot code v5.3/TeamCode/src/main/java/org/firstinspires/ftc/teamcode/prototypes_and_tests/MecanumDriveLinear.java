package org.firstinspires.ftc.teamcode.prototypes_and_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot_systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot_systems.Robot;

import java.util.concurrent.Callable;

@Autonomous(name = "mecanumTest",group = "")

public class MecanumDriveLinear extends LinearOpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive.init(hardwareMap , this);
        robot.init(hardwareMap,this);
        waitForStart();
        mecanumDrive.driveByEncoder(150, 0, 1, 5, new Callable() {
            @Override
            public Object call() throws Exception {
                robot.intake.take();
                return null;
            }
        },0.5);




    }
}
