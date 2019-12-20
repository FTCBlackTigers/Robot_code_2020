package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_systems.Intake;
import org.firstinspires.ftc.teamcode.robot_systems.MovingStoneArm;
import org.firstinspires.ftc.teamcode.robot_systems.Robot;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;
import org.firstinspires.ftc.teamcode.utils.SkystoneDetection;

@Autonomous(name = "Stone Blue", group ="blue")
public class StoneBlue extends LinearOpMode {
    Robot robot = new Robot();
    SkystoneDetection skystoneDetection = new SkystoneDetection();

    @Override
    public void runOpMode() throws InterruptedException {
        SkystoneDetection.SkystonePos skystonePos = SkystoneDetection.SkystonePos.NONE;
        GlobalVariables.reset();
        skystoneDetection.init(hardwareMap);
        robot.init(hardwareMap, this);
        /*while(!isStarted() && !isStopRequested()){
            skystonePos= skystoneDetection.getSkystonePos();
            telemetry.addData("skystone", skystonePos);
            telemetry.update();
        }*/
        waitForStart();
        robot.mecanumDrive.driveByEncoder(20,0,1,1);
        double timeToStop = this.getRuntime() + SkystoneDetection.WAITFORDETECTION;
        while(skystonePos ==SkystoneDetection.SkystonePos.NONE && timeToStop> getRuntime() && opModeIsActive()){
            skystonePos = skystoneDetection.getSkystonePos();
            telemetry.addData("skystone", skystonePos);
            telemetry.update();
        }
        telemetry.addData("skystone:" ,skystonePos);
        switch (skystonePos){
            case NONE:
            case LEFT:
                robot.mecanumDrive.driveByEncoder(50,0,0.7,2);
                break;
            case CENTER:
                robot.mecanumDrive.driveByEncoder(100,40,0.7,2);
                break;
            case RIGHT:
                robot.mecanumDrive.driveByEncoder(120,60,0.7,2);
                break;


        }
        /*robot.mecanumDrive.driveByEncoder(80,180,0.7,2.5);
        robot.intake.moveRampAuto(Intake.RampAngle.ANGLE_DOWN);
        robot.movingStoneArm.moveAngleAuto(MovingStoneArm.ArmAngle.AUTO_GRAB);
        robot.foundationMove.grab();
        robot.mecanumDrive.driveByEncoder(25,0,0.7,1);
        robot.mecanumDrive.turnByGyroAbsolut(90,3);
        robot.mecanumDrive.driveByEncoder(150,180,0.7,3);
        robot.foundationMove.up();
        robot.movingStoneArm.moveAngleAuto(MovingStoneArm.ArmAngle.LOW_POS);
        robot.mecanumDrive.driveByEncoder(80,0,0.7,2);*/
        GlobalVariables.setEndAutoRobotAngle(robot.mecanumDrive.gyro.getAngle()+180);
        GlobalVariables.endAutoArmEncoder = robot.movingStoneArm.motorAngle.getCurrentPosition();
        GlobalVariables.endAutoRampEncoder = robot.intake.rampAngle.getCurrentPosition();
    }
}
