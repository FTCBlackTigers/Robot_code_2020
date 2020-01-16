package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.DashboardConfigValues;
import org.firstinspires.ftc.teamcode.robot_systems.Robot;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;
import org.firstinspires.ftc.teamcode.utils.SkystoneDetection;

import java.util.concurrent.Callable;

@Autonomous(name = "BlueFoundationAndStone", group ="blue")

public class BlueFoundationAndStone extends LinearOpMode {
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
        double timeToStop = this.getRuntime() + SkystoneDetection.WAITFORDETECTION;
        while(skystonePos ==SkystoneDetection.SkystonePos.NONE && timeToStop> getRuntime() && opModeIsActive()){
            skystonePos = skystoneDetection.getSkystonePos();
            telemetry.addData("skystone", skystonePos);
            telemetry.update();
        }
        skystonePos = DashboardConfigValues.skystonePos;
        robot.mecanumDrive.driveByEncoder(15,0,1,1);
        robot.intake.take();
        telemetry.addData("skystone:" ,skystonePos);
        int skystoneAddDistance = 0;
        int rightAddDistance = 0;
        switch (skystonePos){
            case NONE:
            case LEFT:
                robot.mecanumDrive.driveByEncoder(110,-50,0.7,2);
                robot.mecanumDrive.turnByGyroAbsolute(-30,2);
                break;
            case CENTER:
                skystoneAddDistance = 25;
                robot.mecanumDrive.driveByEncoder(40,0,0.7,2);
                robot.mecanumDrive.turnByGyroAbsolute(-30,2);
                break;
            case RIGHT:
                skystoneAddDistance = 35;
                rightAddDistance = 15;
                robot.mecanumDrive.driveByEncoder(65,20,0.7,3);
                robot.mecanumDrive.turnByGyroAbsolute(-30,2);
                break;
        }
        robot.mecanumDrive.driveByEncoder(20,0,0.3,2);
        robot.mecanumDrive.driveByEncoder(20,180,1,2);
        robot.mecanumDrive.turnByGyroAbsolute(-93,5);
        robot.mecanumDrive.driveByEncoder(175 + skystoneAddDistance, 180, 0.8, 10, new Callable() {
            @Override
            public Object call() throws Exception {
                robot.intake.stop();
                return null;
            }
        },1);
        robot.mecanumDrive.turnByGyroAbsolute(180,2);
        robot.mecanumDrive.driveByEncoder(35, 180, 0.5, 2, new Callable() {
            @Override
            public Object call() throws Exception {
                return null;
            }
        },1);
        robot.foundationMove.down();
        sleep(500);
        robot.mecanumDrive.driveByEncoder(120,-60,1,3);
        robot.mecanumDrive.driveByEncoder(40,0,1,1);
        robot.mecanumDrive.turnByGyroAbsolute(-60,3);
        robot.mecanumDrive.driveByEncoder(120, 180,1,3);
        //robot.mecanumDrive.driveByEncoder(60,90,1,1);
        robot.foundationMove.up();
        sleep(700);
        robot.mecanumDrive.driveByEncoder(100,-20,1,2);
        robot.mecanumDrive.driveByEncoder(40,0,0.7,3);
        GlobalVariables.setEndAutoRobotAngle(robot.mecanumDrive.gyro.getAngle());
    }
}