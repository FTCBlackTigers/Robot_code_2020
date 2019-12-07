package org.firstinspires.ftc.teamcode.prototypes_and_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot_systems.MecanumDrive;
@Autonomous(name = "mecanumTest",group = "")
@Disabled
public class MecanumDriveLinear extends LinearOpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive.init(hardwareMap , this);
        waitForStart();
        /*mecanumDrive.driveByEncoder(100,45,0.5 );
        mecanumDrive.driveByEncoder(100,-45,0.5 );
        mecanumDrive.driveByEncoder(100,135,0.5 );
        mecanumDrive.driveByEncoder(100,-135,0.5 );*/
        mecanumDrive.turnByGyroAbsolut(0,10);




    }
}
