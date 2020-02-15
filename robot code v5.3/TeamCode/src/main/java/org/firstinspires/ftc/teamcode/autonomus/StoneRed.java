package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_systems.Lift;
import org.firstinspires.ftc.teamcode.robot_systems.Robot;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;
import org.firstinspires.ftc.teamcode.utils.SkystoneDetection;

import java.util.concurrent.Callable;

@Autonomous(name = "Stone Red", group ="red")
//@Disabled
public class StoneRed extends LinearOpMode {
    private Robot robot = new Robot();
    private SkystoneDetection skystoneDetection = new SkystoneDetection();

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalVariables.reset();
        SkystoneDetection.SkystonePos skystonePos = SkystoneDetection.SkystonePos.NONE;
        skystoneDetection.init(hardwareMap, this);
        robot.init(hardwareMap, this);
        while (!isStarted() && !isStopRequested()) {
            skystonePos = skystoneDetection.getSkystonePos();
            telemetry.addData("skystone", skystonePos);
            telemetry.update();
        }
        double timeToStop = this.getRuntime() + SkystoneDetection.WAIT_FOR_DETECTION;
        while (skystonePos == SkystoneDetection.SkystonePos.NONE && timeToStop > getRuntime() && opModeIsActive()) {
            skystonePos = skystoneDetection.getSkystonePos();
            telemetry.addData("skystone", skystonePos);
            telemetry.update();
        }
        robot.foundationMove.down();
        robot.lift.moveLift(Lift.LiftPosition.RELEASE_INTAKE);
        robot.mecanumDrive.driveByEncoder(15, 0, 1, 1);
        telemetry.addData("skystone:", skystonePos);
        double skystoneAddDistance = 0;
        switch (skystonePos) {
            case NONE:
            case CENTER: //RIGHT
                robot.mecanumDrive.driveByEncoder(120, 55, 0.7, 3);
                break;
            case LEFT: //CENTER
                skystoneAddDistance = 25;
                robot.mecanumDrive.driveByEncoder(85, 40, 0.7, 2);
                break;
            case RIGHT: // LEFT
                skystoneAddDistance = 50;
                robot.mecanumDrive.driveByEncoder(40, -10, 0.7, 3);
                break;
        }
        robot.mecanumDrive.turnByGyroAbsolute(30, 2);
        robot.lift.moveLift(Lift.LiftPosition.READY_TO_TAKE_STONE);
        robot.intake.take();
        robot.mecanumDrive.driveByEncoder(40, 0, 0.3, 2);
        robot.mecanumDrive.turnByGyroRelative(-5, 1);
        robot.mecanumDrive.turnByGyroRelative(5, 1);
        robot.mecanumDrive.driveByEncoder(30, 180, 0.7, 2);
        sleep(500);
        robot.lift.moveLift(Lift.LiftPosition.TAKE_STONE);
        robot.mecanumDrive.turnByGyroAbsolute(93, 5);
        robot.mecanumDrive.driveByEncoder(90 + skystoneAddDistance, 180, 1, 2, new Callable() {
            @Override
            public Object call() throws Exception {
                robot.intake.stop();
                robot.lift.closeGrabServo();
                return null;
            }
        }, 0.7);
        robot.lift.moveHeightAuto(Lift.LiftPosition.LEVEL2);
        robot.lift.moveRangeOutAuto(Lift.LiftPosition.LEVEL1);
        robot.lift.openGrabServo();
        robot.lift.moveLiftAuto(Lift.LiftPosition.TAKE_STONE);
        robot.mecanumDrive.turnByGyroAbsolute(90, 1);
        robot.mecanumDrive.driveByEncoder(125 + skystoneAddDistance, 0, 1, 2);
        robot.mecanumDrive.driveByEncoder(40, 90, 0.7, 2);
        robot.mecanumDrive.turnByGyroAbsolute(70, 2);
        robot.lift.moveLift(Lift.LiftPosition.READY_TO_TAKE_STONE);
        robot.intake.take();
        robot.mecanumDrive.driveByEncoder(20, 0, 0.3, 2);
        robot.mecanumDrive.driveByEncoder(100, -110, 1, 2);
        robot.mecanumDrive.turnByGyroAbsolute(90, 2);
        robot.mecanumDrive.driveByEncoder(150 + skystoneAddDistance, 180, 1, 2, new Callable() {
            @Override
            public Object call() throws Exception {
                robot.intake.stop();
                robot.lift.moveLift(Lift.LiftPosition.TAKE_STONE);
                return null;
            }
        }, 0.1);
        robot.lift.closeGrabServo();
        sleep(500);
        robot.lift.moveHeightAuto(Lift.LiftPosition.LEVEL2);
        robot.lift.moveRangeOutAuto(Lift.LiftPosition.LEVEL1);
        robot.lift.openGrabServo();
        sleep(500);
        robot.lift.moveHeightAuto(Lift.LiftPosition.TAKE_STONE);
        robot.lift.moveLift(Lift.LiftPosition.TAKE_STONE);
        robot.mecanumDrive.turnByGyroAbsolute(90, 1);
        robot.mecanumDrive.driveByEncoder(50, 0, 0.7, 2);
        robot.intake.stop();
        robot.endAuto(0);
    }
}