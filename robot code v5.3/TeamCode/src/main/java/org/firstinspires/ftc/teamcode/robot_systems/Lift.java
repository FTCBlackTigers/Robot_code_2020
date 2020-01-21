package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;

public class Lift extends SubSystem{
    public enum LiftPosition{
        TAKE_STONE(0,0), LEVEL1(0,36), LEVEL2(8,36), LEVEL3(14,36),
        ABOVE_FOUNDATION(6,35),MOVE_OUT_HIGHT(7,0);


        private double height;
        private double tickPerCMVertical = 398.33;
        private double rangeOut;
        private double tickPerCMHorizontal = 69.78;
        LiftPosition (double height, double rangeOut) {
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
    public enum LiftState{
        TAKE_STONE, MOVE_UP, MOVE_OUT,MOVE_TO_LEVEL, AT_LEVEL, RELEASE;
    }
    DcMotor liftMotorVertical;
    DcMotor liftMotorHorizontal;
    Servo grabServo;
    private final double POWER = 1;
    private final double GRAB_POS = 0;
    private final double RELEASED_POS = 0.15;

    LiftPosition[]positions = {LiftPosition.LEVEL1,
                                  LiftPosition.LEVEL2,
                                 LiftPosition.LEVEL3};
    private int currentPositionHeight = -1;
    private LiftState currentState;

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
        currentState = LiftState.TAKE_STONE;
    }

    @Override
    public void teleopMotion(Controller driver, Controller operator) {
        if(operator.x.onClick()){
            openGrabServo();
            moveLift(LiftPosition.TAKE_STONE);
            currentPositionHeight = -1;
            currentState = LiftState.TAKE_STONE;
        }
        if(operator.a.onClick()){
            openGrabServo();
        }
        if(operator.b.onClick()){
            closeGrabServo();
        }
        if(operator.dpadUp.onClick()){
            if(currentState == LiftState.TAKE_STONE){
                currentPositionHeight = 1;
                moveHight(LiftPosition.MOVE_OUT_HIGHT);
            }
            else{
                currentPositionHeight++;
                currentPositionHeight = Range.clip(currentPositionHeight,0, positions.length-1);
                //moveLift(positions[currentPositionHeight]);
            }
            currentState = LiftState.MOVE_UP;
        }
        if(operator.dpadDown.onClick()){
            if(currentState == LiftState.TAKE_STONE){
                currentPositionHeight = 1;
                moveHight(LiftPosition.MOVE_OUT_HIGHT);
            }
            else{
                currentPositionHeight--;
                currentPositionHeight = Range.clip(currentPositionHeight,0, positions.length-1);
                //moveLift(positions[currentPositionHeight]);
            }
            currentState = LiftState.MOVE_UP;
        }
        switch (currentState){
            case TAKE_STONE:
                //TODO: check how to recoginze a stone
                break;
            case MOVE_UP:
                if (liftMotorVertical.getCurrentPosition()>= LiftPosition.ABOVE_FOUNDATION.getHeight()||
                        liftMotorHorizontal.getCurrentPosition()>= LiftPosition.ABOVE_FOUNDATION.getRangeOut()){
                    currentState = LiftState.MOVE_OUT;
                    moveRangeOut(positions[currentPositionHeight]);
                }
                break;
            case MOVE_OUT:
                if ((liftMotorVertical.getCurrentPosition()>= LiftPosition.ABOVE_FOUNDATION.getHeight() &&
                        positions[currentPositionHeight].getHeight()>= LiftPosition.ABOVE_FOUNDATION.getHeight())||
                        liftMotorHorizontal.getCurrentPosition()>= LiftPosition.ABOVE_FOUNDATION.getRangeOut()){
                    currentState = LiftState.MOVE_TO_LEVEL;
                    moveLift(positions[currentPositionHeight]);
                }
                break;
            case MOVE_TO_LEVEL:
                if (Math.abs(liftMotorHorizontal.getTargetPosition() - liftMotorHorizontal.getCurrentPosition()) < 75 &&
                        Math.abs(liftMotorVertical.getTargetPosition() - liftMotorVertical.getCurrentPosition()) < 75){
                    liftMotorVertical.setPower(0);
                    liftMotorHorizontal.setPower(0);
                    currentState = LiftState.AT_LEVEL;
                }
                break;
            case AT_LEVEL:
                break;
        }
        opMode.telemetry.addLine("LIFT");
        opMode.telemetry.addData("\tCurrent state", currentState);
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
        moveHight(position);
        moveRangeOut(position);
    }
    public void moveHight(LiftPosition position){
        liftMotorVertical.setTargetPosition(position.getHeight());
        liftMotorVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorVertical.setPower(Math.abs(POWER));
    }
    public void moveRangeOut(LiftPosition position){
        liftMotorHorizontal.setTargetPosition(position.getRangeOut());
        liftMotorHorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorHorizontal.setPower(POWER);
    }
}