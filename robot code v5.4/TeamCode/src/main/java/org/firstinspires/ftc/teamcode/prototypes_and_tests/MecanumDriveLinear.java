package org.firstinspires.ftc.teamcode.prototypes_and_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot_systems.Robot;

@Autonomous(name = "mecanumTest",group = "")
@Disabled

public class MecanumDriveLinear extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive.init(hardwareMap , this);
        robot.init(hardwareMap,this);
        waitForStart();
        //mecanumDrive.driveByEncoder(DashboardConfigValues.distance,DashboardConfigValues.angle,DashboardConfigValues.power,5 );
        mecanumDrive.driveByEncoder(100, 150, 1, 2);
        mecanumDrive.driveByEncoder(100, 210, 1, 2);
    }
}
