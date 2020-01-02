package org.firstinspires.ftc.teamcode.prototypes_and_tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot_systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot_systems.Robot;

@Autonomous(name = "mecanumTest",group = "")
@Disabled
public class MecanumDriveLinear extends LinearOpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive.init(hardwareMap , this);
        robot.init(hardwareMap,this);
        waitForStart();
        robot.foundationMove.down();
        sleep(1000);
        //mecanumDrive.driveByEncoder(100,90,0.5 ,2);
        mecanumDrive.driveByEncoder(100,60,1,2 );
        mecanumDrive.driveByEncoder(100,45,1 ,2 );
        //mecanumDrive.driveByEncoder(100,10,0.5,2);




    }
}
