package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_systems.Robot;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;
import org.firstinspires.ftc.teamcode.utils.SkystoneDetection;

import java.util.concurrent.Callable;

@Autonomous(name = "Stone Blue", group ="blue")
public class StoneBlue extends LinearOpMode {
    private Robot robot = new Robot();
    private SkystoneDetection skystoneDetection = new SkystoneDetection();

    @Override
    public void runOpMode() throws InterruptedException {
        SkystoneDetection.SkystonePos skystonePos = SkystoneDetection.SkystonePos.NONE;
        GlobalVariables.reset();
        skystoneDetection.init(hardwareMap);
        robot.init(hardwareMap, this);
        while(!isStarted() && !isStopRequested()){
            skystonePos= skystoneDetection.getSkystonePos();
            telemetry.addData("skystone", skystonePos);
            telemetry.update();
        }
        //waitForStart();
        //robot.mecanumDrive.driveByEncoder(20,0,0.3,1);
        double timeToStop = this.getRuntime() + SkystoneDetection.WAITFORDETECTION;
        while(skystonePos ==SkystoneDetection.SkystonePos.NONE && timeToStop> getRuntime() && opModeIsActive()){
            skystonePos = skystoneDetection.getSkystonePos();
            telemetry.addData("skystone", skystonePos);
            telemetry.update();
        }
        robot.mecanumDrive.driveByEncoder(15,0,1,1);
        robot.intake.take();
        telemetry.addData("skystone:" ,skystonePos);
        int skystoneAddDistance = 0;
        int rightAddDistance = 0;
        switch (skystonePos){
            case NONE:
            case LEFT:
                robot.mecanumDrive.driveByEncoder(70,-40,0.7,2);
                robot.mecanumDrive.turnByGyroAbsolute(-20,2);
                break;
            case CENTER:
                skystoneAddDistance = 25;
                robot.mecanumDrive.driveByEncoder(30,0,0.7,2);
                robot.mecanumDrive.turnByGyroAbsolute(-30,2);
                break;
            case RIGHT:
                skystoneAddDistance = 35;
                rightAddDistance = 15;
                robot.mecanumDrive.driveByEncoder(80,30,0.7,2);
                robot.mecanumDrive.turnByGyroAbsolute(-30,2);
                break;
        }
        robot.mecanumDrive.driveByEncoder(50,0,0.3,2);
        robot.intake.stop();
        robot.mecanumDrive.driveByEncoder(60,180,0.7,2);
        robot.mecanumDrive.turnByGyroAbsolute(90,2);
        robot.mecanumDrive.driveByEncoder(80 + skystoneAddDistance,0,1,2);
        robot.intake.reverse();
        sleep(500);
        robot.mecanumDrive.driveByEncoder(145 + skystoneAddDistance, 180, 1, 2);
        robot.intake.take();
        robot.mecanumDrive.turnByGyroAbsolute(-30,2);
        robot.mecanumDrive.driveByEncoder(50,0,0.3,2);
        robot.mecanumDrive.driveByEncoder(70,180,0.7,2);
        robot.intake.stop();
        robot.mecanumDrive.turnByGyroAbsolute(90,2);
        robot.mecanumDrive.driveByEncoder(150 + skystoneAddDistance - rightAddDistance,0,1,2);
        robot.intake.reverse();
        sleep(500);
        robot.mecanumDrive.driveByEncoder(50,180,0.7,2);
        robot.intake.stop();
        GlobalVariables.setEndAutoRobotAngle(robot.mecanumDrive.gyro.getAngle());
       // GlobalVariables.endAutoArmEncoder = robot.movingStoneArm.motorAngle.getCurrentPosition();
        //GlobalVariables.endAutoRampEncoder = robot.intake.rampAngle.getCurrentPosition();
    }
}
