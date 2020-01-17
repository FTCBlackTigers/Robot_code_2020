package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;

import java.util.Timer;

public class Lift extends SubSystem{
    enum LiftPosition{
        TAKE_STONE(0,0), LEVEL1(7,0.5), LEVEL2(21,0.5), LEVEL3(35,0.5) ,
        LEVEL1_LONG(7,1) , LEVEL2_LONG(21,1) , LEVEL3_LONG(35, 1);

        private double height;
        //TODO CHACK tickperCM
        private double tickPerCMVertical =1;
        private double rangeOut;
        private int tickPerCMHorizontal = 1;
        private LiftPosition (double height, double rangeOut) {
            this.height = height;
            this.rangeOut = rangeOut;
        }
        public int getHeight() {
             return (int) (height*tickPerCMVertical);
        }

        public int getRangeOut() {
            return(int) (rangeOut * tickPerCMHorizontal);
        }
    }

    DcMotor liftMotorVertical;
    DcMotor liftMotorHorizontal;
    Servo grabServo;
    int servoPosition = 0;
    private final double POWER = 1;
    private final double GRAB_POS = 0;
    private final double RELEASED_POS = 0.15;
    private final Timer t = new java.util.Timer();

    LiftPosition[][]positions = {{LiftPosition.LEVEL1,LiftPosition.LEVEL1_LONG},
                                  {LiftPosition.LEVEL2,LiftPosition.LEVEL2_LONG},
                                 {LiftPosition.LEVEL3,LiftPosition.LEVEL3_LONG}};
    private int currentPositionHeight = -1;
    private int currentPositionLength = -1;

    @Override
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode=opMode;
        liftMotorVertical = hardwareMap.get(DcMotor.class, "liftMotorVertical");
        liftMotorHorizontal = hardwareMap.get(DcMotor.class, "liftMotorHorizontal");
        grabServo = hardwareMap.get(Servo.class, "grabServo");
        liftMotorVertical.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotorVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorHorizontal.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotorHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void teleopMotion(Controller driver, Controller operator) {
        if(operator.x.onClick()){
            openGrabServo();
            moveLift(LiftPosition.TAKE_STONE);
            currentPositionHeight = -1;
            currentPositionLength = -1;
        }
        if(operator.a.onClick()){
            openGrabServo();
        }
        if(operator.b.onClick()){
            closeGrabServo();
        }
        if(operator.dpadUp.onClick()){
            currentPositionHeight++;
            currentPositionHeight = Range.clip(currentPositionHeight,0, positions.length-1);
            currentPositionLength = Range.clip(currentPositionLength,0, positions[0].length-1);
            moveLift(positions[currentPositionHeight][currentPositionLength]);
        }
        if(operator.dpadDown.onClick()){
            currentPositionHeight--;
            currentPositionHeight = Range.clip(currentPositionHeight,0, positions.length-1);
            currentPositionLength = Range.clip(currentPositionLength,0, positions[0].length-1);
            moveLift(positions[currentPositionHeight][currentPositionLength]);
        }
        if(operator.dpadLeft.onClick()){
            currentPositionLength++;
            currentPositionHeight = Range.clip(currentPositionHeight,0, positions.length-1);
            currentPositionLength = Range.clip(currentPositionLength,0, positions[0].length-1);
            moveLift(positions[currentPositionHeight][currentPositionLength]);
        }
        if(operator.dpadRight.onClick()){
            currentPositionLength--;
            currentPositionHeight = Range.clip(currentPositionHeight,0, positions.length-1);
            currentPositionLength = Range.clip(currentPositionLength,0, positions[0].length-1);
            moveLift(positions[currentPositionHeight][currentPositionLength]);
        }
        opMode.telemetry.addLine("LIFT");
        opMode.telemetry.addData("\ttarget Vertical ",  liftMotorVertical.getTargetPosition());
        opMode.telemetry.addData("\tcurrent position Vertical", liftMotorVertical.getCurrentPosition());
        opMode.telemetry.addData("\ttarget Horizontal ", liftMotorHorizontal.getTargetPosition());
        opMode.telemetry.addData("\tcurrent position Horizontal", liftMotorHorizontal.getCurrentPosition());
    }
    public void manualTeleop(Controller operator){
        if(operator.a.onClick()){
            openGrabServo();
        }
        if(operator.b.onClick()){
            closeGrabServo();
        }
        liftMotorVertical.setPower(operator.leftStickY.getValue());
        liftMotorHorizontal.setPower(operator.rightStickY.getValue());
        opMode.telemetry.addLine("LIFT");
        opMode.telemetry.addData("\ttarget Vertical ",  liftMotorVertical.getTargetPosition());
        opMode.telemetry.addData("\tcurrent position Vertical", liftMotorVertical.getCurrentPosition());
        opMode.telemetry.addData("\ttarget Horizontal ", liftMotorHorizontal.getTargetPosition());
        opMode.telemetry.addData("\tcurrent position Horizontal", liftMotorHorizontal.getCurrentPosition());
    }
    public void closeGrabServo(){
        grabServo.setPosition(GRAB_POS);
    }
    public void openGrabServo (){
        grabServo.setPosition(RELEASED_POS);
    }
    public void moveLift(LiftPosition position) {
        liftMotorHorizontal.setTargetPosition(position.getRangeOut());
        liftMotorHorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorHorizontal.setPower(POWER);
        liftMotorVertical.setTargetPosition(position.getHeight() - GlobalVariables.endAutoArmEncoder);
        liftMotorVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorVertical.setPower(Math.abs(POWER));
    }
}