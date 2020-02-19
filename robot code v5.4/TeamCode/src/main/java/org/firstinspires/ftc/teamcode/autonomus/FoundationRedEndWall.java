package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_systems.Lift;
import org.firstinspires.ftc.teamcode.robot_systems.Robot;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;

@Autonomous(name = "RED Foundation Wall", group ="red")
public class FoundationRedEndWall extends LinearOpMode {
    private Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalVariables.reset();
        robot.init(hardwareMap, this);
        waitForStart();
        robot.lift.moveLift(Lift.LiftPosition.RELEASE_INTAKE);
        robot.mecanumDrive.driveByEncoder(150, 210, 1, 3);
        robot.mecanumDrive.driveByEncoder(20, 180, 0.5, 1);
        robot.mecanumDrive.driveByEncoder(15, 180, 0.1, 2);
        robot.foundationMove.down();
        sleep(500);
        robot.mecanumDrive.driveByEncoder(20, 0, 0.7, 1);
        robot.mecanumDrive.turnByGyroAbsolute(-90, 3);
        robot.mecanumDrive.driveByEncoder(40, 180, 0.7, 2);
        robot.foundationMove.up();
        sleep(700);
        robot.lift.moveLift(Lift.LiftPosition.TAKE_STONE);
        robot.mecanumDrive.driveByEncoder(70, -90, 0.5, 3);
        robot.mecanumDrive.driveByEncoder(120, 0, 1, 2);
        robot.endAuto(180);
    }
}
