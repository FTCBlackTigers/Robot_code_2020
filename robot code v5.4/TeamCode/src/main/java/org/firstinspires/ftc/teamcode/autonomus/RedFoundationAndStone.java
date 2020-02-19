package org.firstinspires.ftc.teamcode.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_systems.Lift;
import org.firstinspires.ftc.teamcode.robot_systems.Robot;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;
import org.firstinspires.ftc.teamcode.utils.SkystoneDetection;

import java.util.concurrent.Callable;

@Autonomous(name = "RED foundation And Stone", group ="red")

public class RedFoundationAndStone extends LinearOpMode {
    private Robot robot = new Robot();
    private SkystoneDetection skystoneDetection = new SkystoneDetection();

    @Override
    public void runOpMode() throws InterruptedException {
        SkystoneDetection.SkystonePos skystonePos = SkystoneDetection.SkystonePos.NONE;
        GlobalVariables.reset();
        skystoneDetection.init(hardwareMap, this);
        robot.init(hardwareMap, this);
        while (!isStarted() && !isStopRequested()) {
            skystonePos = skystoneDetection.getSkystonePos();
            telemetry.addData("skystone", skystonePos);
            telemetry.update();
        }
        robot.lift.moveLift(Lift.LiftPosition.RELEASE_INTAKE);
        double timeToStop = this.getRuntime() + SkystoneDetection.WAIT_FOR_DETECTION;
        while (skystonePos == SkystoneDetection.SkystonePos.NONE && timeToStop > getRuntime() && opModeIsActive()) {
            skystonePos = skystoneDetection.getSkystonePos();
            telemetry.addData("skystone", skystonePos);
            telemetry.update();
        }
        robot.mecanumDrive.driveByEncoder(15, 0, 1, 1);
        telemetry.addData("skystone:", skystonePos);
        double skystoneAddDistance = 0;
        switch (skystonePos) {
            case NONE:
            case CENTER: //RIGHT
                robot.mecanumDrive.driveByEncoder(50, 90, 0.5, 3);
                robot.mecanumDrive.driveByEncoder(40, 0, 0.5, 3);
                break;
            case LEFT: //CENTER
                skystoneAddDistance = 25;
                robot.mecanumDrive.driveByEncoder(20, 90, 0.5, 3);
                robot.mecanumDrive.driveByEncoder(40, 0, 0.5, 3);
                break;
            case RIGHT: // LEFT
                skystoneAddDistance = 50;
                robot.mecanumDrive.driveByEncoder(45, -10, 0.7, 3);
                break;
        }
        robot.mecanumDrive.turnByGyroAbsolute(30, 2);
        robot.lift.moveLift(Lift.LiftPosition.READY_TO_TAKE_STONE);
        robot.intake.take();
        robot.mecanumDrive.driveByEncoder(35, 0, 0.3, 2);
        robot.mecanumDrive.driveByEncoder(25, 180, 1, 2);
        sleep(500);
        robot.lift.moveLift(Lift.LiftPosition.TAKE_STONE);
        robot.mecanumDrive.turnByGyroAbsolute(90, 5);
        robot.mecanumDrive.driveByEncoder(162 + skystoneAddDistance, 180, 0.8, 10, new Callable() {
            @Override
            public Object call() throws Exception {
                robot.intake.stop();
                robot.lift.closeGrabServo();
                return null;
            }
        }, 1);
        robot.mecanumDrive.turnByGyroAbsolute(180, 2);
        robot.lift.moveHeight(Lift.LiftPosition.LEVEL2);
        robot.mecanumDrive.driveByEncoder(35, 180, 0.3, 2);
        robot.foundationMove.down();
        robot.lift.moveRangeOutAuto(Lift.LiftPosition.LEVEL2);
        robot.lift.moveLiftAuto(Lift.LiftPosition.LEVEL1);
        robot.lift.openGrabServo();
        sleep(500);
        robot.lift.moveLift(Lift.LiftPosition.TAKE_STONE);
        robot.mecanumDrive.driveByEncoder(45, 0, 1, 3);
        robot.mecanumDrive.turnByGyroAbsolute(90, 3);
        robot.mecanumDrive.driveByEncoder(30, 180, 1, 3);
        robot.foundationMove.up();
        sleep(500);
        robot.mecanumDrive.turnByGyroAbsolute(90, 1);
        robot.mecanumDrive.driveByEncoder(40, 70, 0.5, 2);
        robot.mecanumDrive.driveByEncoder(100, 0, 1, 2);
        robot.endAuto(0);
    }
}
