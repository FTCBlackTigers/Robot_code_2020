package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_systems.Intake;
import org.firstinspires.ftc.teamcode.robot_systems.Robot;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;

@Autonomous(name = "Red foundation", group ="red")
public class FoundationRed extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalVariables.reset();
        robot.init(hardwareMap, this);
        waitForStart();
        robot.mecanumDrive.driveByEncoder(70,180,1, 1.5 );
        robot.mecanumDrive.driveByEncoder(50, -90,0.3, 1);
        robot.mecanumDrive.driveByEncoder(25,180,0.1,2);
        robot.foundationMove.down();
        sleep(500);
        robot.mecanumDrive.turnByGyroAbsolut(160, 1);
        robot.mecanumDrive.turnByGyroAbsolut(-160, 3);
        robot.mecanumDrive.driveByEncoder(60, 180,0.5, 2);
        robot.foundationMove.up();
        sleep(700);
        robot.mecanumDrive.driveByEncoder(10,0,0.5,1);
        robot.mecanumDrive.turnByGyroAbsolut(-95,3);
        robot.intake.moveRampAuto(Intake.RampAngle.ANGLE_DOWN);
        robot.mecanumDrive.driveByEncoder(120,0,1,2);
        GlobalVariables.setEndAutoRobotAngle(robot.mecanumDrive.gyro.getAngle()+180);
        GlobalVariables.endAutoArmEncoder = robot.movingStoneArm.motorAngle.getCurrentPosition();
        GlobalVariables.endAutoRampEncoder = robot.intake.rampAngle.getCurrentPosition();
    }
}
