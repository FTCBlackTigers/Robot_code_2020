package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_systems.Intake;
import org.firstinspires.ftc.teamcode.robot_systems.Robot;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;

@Autonomous(name = "Blue foundation", group ="blue")
public class FoundationBlue extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalVariables.reset();
        robot.init(hardwareMap, this);
        waitForStart();
        robot.mecanumDrive.driveByEncoder(110,150,1, 1.5 );
        robot.mecanumDrive.driveByEncoder(20,180,1,1);
        robot.mecanumDrive.driveByEncoder(10,180,0.1,2);
        robot.foundationMove.down();
        sleep(500);
        robot.mecanumDrive.driveByEncoder(150,-60,0.7,1);
        robot.mecanumDrive.turnByGyroAbsolut(90,3);
        //robot.mecanumDrive.driveByEncoder(70,110,0.7,1);
        robot.mecanumDrive.driveByEncoder(40, 180,0.7,3);
        robot.mecanumDrive.driveByEncoder(40,90,0.7,1);
        robot.foundationMove.up();
        sleep(700);
        //robot.intake.moveRampAuto(Intake.RampAngle.ANGLE_DOWN);
        robot.mecanumDrive.driveByEncoder(120,0,1,2);;
        GlobalVariables.setEndAutoRobotAngle(robot.mecanumDrive.gyro.getAngle()+180);
        //GlobalVariables.endAutoArmEncoder = robot.movingStoneArm.motorAngle.getCurrentPosition();
        //GlobalVariables.endAutoRampEncoder = robot.intake.rampAngle.getCurrentPosition();
    }
}
