package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_systems.Robot;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;

@Autonomous(name = "Red foundation", group ="red")
public class FoundationRed extends LinearOpMode {
    private Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalVariables.reset();
        robot.init(hardwareMap, this);
        waitForStart();
        robot.mecanumDrive.driveByEncoder(120,210,1, 1.5 );
        robot.mecanumDrive.driveByEncoder(20,180,0.5,1);
        robot.mecanumDrive.driveByEncoder(10,180,0.1,2);
        robot.foundationMove.down();
        sleep(500);
        robot.mecanumDrive.turnByGyroAbsolute(-30,3);
        robot.mecanumDrive.driveByEncoder(70, 0,0.7,3);
        robot.mecanumDrive.turnByGyroAbsolute(-90,2);
        robot.mecanumDrive.driveByEncoder(40,180,0.7,2);
        robot.foundationMove.up();
        sleep(700);
        //robot.intake.moveRampAuto(Intake.RampAngle.ANGLE_DOWN);
        robot.mecanumDrive.driveByEncoder(120,0,1,2);
        GlobalVariables.setEndAutoRobotAngle(robot.mecanumDrive.getAngle()+180);
        //GlobalVariables.endAutoArmEncoder = robot.movingStoneArm.motorAngle.getCurrentPosition();
        //GlobalVariables.endAutoRampEncoder = robot.intake.rampAngle.getCurrentPosition();
    }
}
