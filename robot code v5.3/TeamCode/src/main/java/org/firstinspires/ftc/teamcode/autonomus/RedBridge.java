package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_systems.Robot;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;

@Autonomous(name = "RED Bridge", group = "red")
public class RedBridge extends LinearOpMode {

    private final int WAIT = 0;
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalVariables.reset();
        robot.init(hardwareMap, this);
        waitForStart();
        sleep(WAIT);
        robot.mecanumDrive.driveByEncoder(25, 180, 0.5, 10);
        robot.endAuto(90);
    }
}
