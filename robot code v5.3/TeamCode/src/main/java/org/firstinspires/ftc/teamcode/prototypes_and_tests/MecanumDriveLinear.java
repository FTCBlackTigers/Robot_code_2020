package org.firstinspires.ftc.teamcode.prototypes_and_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot_systems.MecanumDrive;
@Autonomous(name = "mecanumTest",group = "")
public class MecanumDriveLinear extends LinearOpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive.init(hardwareMap , this);
        waitForStart();
       mecanumDrive.driveByEncoder(100,0,0.5 );
        mecanumDrive.driveByEncoder(100,90,0.5 );
        mecanumDrive.driveByEncoder(100,180,0.5 );
        mecanumDrive.driveByEncoder(100,-90,0.5 );
        mecanumDrive.turnByGyroAbsolut(0,3);




    }
}
