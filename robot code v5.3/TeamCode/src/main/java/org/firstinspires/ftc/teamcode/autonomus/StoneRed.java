package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_systems.Intake;
import org.firstinspires.ftc.teamcode.robot_systems.MovingStoneArm;
import org.firstinspires.ftc.teamcode.robot_systems.Robot;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;

@Autonomous(name = "Stone Red", group ="red")
@Disabled
public class StoneRed extends LinearOpMode {
    private Robot robot = new Robot();

    @Override
   public void runOpMode() throws InterruptedException {
        GlobalVariables.reset();
        robot.init(hardwareMap, this);
        waitForStart();

        /*robot.mecanumDrive.driveByEncoder(80,180,0.7,2.5);
        robot.intake.moveRampAuto(Intake.RampAngle.ANGLE_DOWN);
        robot.movingStoneArm.moveAngleAuto(MovingStoneArm.ArmAngle.AUTO_GRAB);
        robot.foundationMove.grab();
        robot.mecanumDrive.driveByEncoder(25,0,0.7,1);
        robot.mecanumDrive.turnByGyroAbsolute(-90,3);
        robot.mecanumDrive.driveByEncoder(150,180,0.7,3);
        robot.foundationMove.up();
        robot.movingStoneArm.moveAngleAuto(MovingStoneArm.ArmAngle.LOW_POS);
        robot.mecanumDrive.driveByEncoder(80,0,0.7,2);
        GlobalVariables.setEndAutoRobotAngle(robot.mecanumDrive.gyro.getAngle()+180);
        GlobalVariables.endAutoArmEncoder = robot.movingStoneArm.motorAngle.getCurrentPosition();
        GlobalVariables.endAutoLiftVertical = robot.intake.rampAngle.getCurrentPosition();*/
    }
}