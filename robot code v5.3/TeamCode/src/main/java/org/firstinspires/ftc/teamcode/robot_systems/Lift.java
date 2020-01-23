package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.controller.Button;
import org.firstinspires.ftc.teamcode.controller.Controller;

public class Lift extends SubSystem{
    public enum LiftPosition{
        TAKE_STONE(1, 0), MOVE_OUT_HEIGHT(7,0),
        LEVEL1(0,36), LEVEL2(8,36), LEVEL3(14,36),
        ABOVE_FOUNDATION(6,35), READY_TO_TAKE_STONE(6,0),
        MAX_BOUNDARY(15, 36), MIN_BOUNDARY(1, 0);

        private double height;
        private final double tickPerCMVertical = 398.33;
        private double rangeOut;
        private final double tickPerCMHorizontal = 69.78;

        LiftPosition (double height, double rangeOut) {
            this.height = height;
            this.rangeOut = rangeOut;
        }

        public int getHeight() {
             return (int) (height * tickPerCMVertical);
        }

        public int getRangeOut() {
            return(int) (rangeOut * tickPerCMHorizontal);
        }
    }

    public enum LiftState{
        TAKE_STONE, MOVE_UP, MOVE_OUT,MOVE_TO_LEVEL, AT_LEVEL, MANUAL;
    }

    private DcMotor liftMotorVertical;
    private DcMotor liftMotorHorizontal;
    private Servo grabServo;
    private DigitalChannel resetTouchSensor;

    private static final double POWER = 1;
    private static final double GRAB_POS = 0;
    private static final double RELEASE_POS = 0.25;
    private static final double RELEASE_ON_FOUNDATION = 0.2;

    private final LiftPosition[] positions = {LiftPosition.LEVEL1, LiftPosition.LEVEL2, LiftPosition.LEVEL3};
    private int targetLevel = -1;

    private LiftState currentState;
    private Button resetEncoderButton = new Button();

    @Override
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode=opMode;

        liftMotorVertical = hardwareMap.get(DcMotor.class, "liftMotorVertical");
        liftMotorHorizontal = hardwareMap.get(DcMotor.class, "liftMotorHorizontal");
        grabServo = hardwareMap.get(Servo.class, "grabServo");
        resetTouchSensor = hardwareMap.get(DigitalChannel.class, "liftReset");

        liftMotorVertical.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotorHorizontal.setDirection(DcMotorSimple.Direction.FORWARD);

        liftMotorVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetTouchSensor.setMode(DigitalChannel.Mode.INPUT);

        openGrabServo();

        currentState = LiftState.TAKE_STONE;
    }

    @Override
    public void teleopMotion(Controller driver, Controller operator) {
        resetEncoderButton.setState(resetTouchSensor.getState());
        if(resetEncoderButton.onClick()){
            liftMotorHorizontal.setPower(0);
            liftMotorHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotorHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            currentState = LiftState.TAKE_STONE;
        }

        if(operator.x.onClick()){
            openGrabServo();
            moveLift(LiftPosition.TAKE_STONE);
            targetLevel = -1;
            currentState = LiftState.TAKE_STONE;
        }
        if(operator.a.onClick()){
            if (currentState == LiftState.TAKE_STONE){
                openGrabServo();
            }
            else {
                releaseOnFoundation();
            }
        }
        if(operator.b.onClick()){
            closeGrabServo();
        }
        if(operator.dpadUp.onClick()){
            if(currentState == LiftState.TAKE_STONE){
                targetLevel = 0;
                moveHeight(LiftPosition.MOVE_OUT_HEIGHT);
            }
            else{
                targetLevel++;
                targetLevel = Range.clip(targetLevel,0, positions.length - 1);
            }
            currentState = LiftState.MOVE_UP;
        }
        if(operator.dpadDown.onClick()){
            if(currentState == LiftState.TAKE_STONE){
                targetLevel = 0;
                moveHeight(LiftPosition.MOVE_OUT_HEIGHT);
            }
            else{
                targetLevel--;
                targetLevel = Range.clip(targetLevel,0, positions.length-1);
            }
            currentState = LiftState.MOVE_UP;
        }
        if(operator.rightStickY.isPressed()){
            manualHorizontalMove(operator.rightStickY.getValue());
        }
        else if(operator.rightStickY.onRealese()){
          manualHorizontalMove(0);
        }
        if(operator.leftStickY.isPressed()) {
            manualVerticalMove(operator.leftStickY.getValue());
        }
        else if(operator.leftStickY.onRealese()){
            manualVerticalMove(0);
        }
        switch (currentState){
            case TAKE_STONE:
                //TODO: check how to recognize a stone
                if (operator.leftTrigger.onClick()){
                    moveLift(LiftPosition.READY_TO_TAKE_STONE);
                }
                else if (operator.leftTrigger.onRealese()){
                    moveLift(LiftPosition.TAKE_STONE);
                }
                break;
            case MOVE_UP:
                if (liftMotorVertical.getCurrentPosition() >= LiftPosition.ABOVE_FOUNDATION.getHeight() ||
                        liftMotorHorizontal.getCurrentPosition() >= LiftPosition.ABOVE_FOUNDATION.getRangeOut()){
                    currentState = LiftState.MOVE_OUT;
                    moveRangeOut(positions[targetLevel]);
                }
                break;
            case MOVE_OUT:
                if ((liftMotorVertical.getCurrentPosition() >= LiftPosition.ABOVE_FOUNDATION.getHeight() &&
                        positions[targetLevel].getHeight() >= LiftPosition.ABOVE_FOUNDATION.getHeight()) ||
                        liftMotorHorizontal.getCurrentPosition() >= LiftPosition.ABOVE_FOUNDATION.getRangeOut()){
                    currentState = LiftState.MOVE_TO_LEVEL;
                    moveLift(positions[targetLevel]);
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
        resetEncoderButton.setPrevState();
    }
    public void manualTeleop(Controller operator){
        resetEncoderButton.setState(resetTouchSensor.getState());
        if(resetEncoderButton.onClick()){
            liftMotorHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotorHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
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
        resetEncoderButton.setPrevState();
    }
    public int getHorizontalPosition(){
       return liftMotorHorizontal.getCurrentPosition();
    }
    public int getVerticalPosition(){
        return liftMotorVertical.getCurrentPosition();
    }
    public void closeGrabServo(){
        grabServo.setPosition(GRAB_POS);
    }
    public void openGrabServo (){
        grabServo.setPosition(RELEASE_POS);
    }
    public void releaseOnFoundation(){
        grabServo.setPosition(RELEASE_ON_FOUNDATION);
    }
    private void manualHorizontalMove(double power){
        currentState = LiftState.MANUAL;
        liftMotorHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if((resetTouchSensor.getState() && power < 0) ||
                (liftMotorHorizontal.getCurrentPosition() > LiftPosition.MAX_BOUNDARY.getRangeOut() && power > 0)){
            liftMotorHorizontal.setPower(0);
            return;
        }
        liftMotorHorizontal.setPower(power);
    }
    private void manualVerticalMove(double power) {
        currentState = LiftState.MANUAL;
        liftMotorVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if ((liftMotorVertical.getCurrentPosition() < LiftPosition.MIN_BOUNDARY.getHeight() && power < 0) ||
                (liftMotorVertical.getCurrentPosition() > LiftPosition.MAX_BOUNDARY.getHeight() && power > 0)) {
            liftMotorVertical.setPower(0);
            return;
        }
        liftMotorVertical.setPower(power);
    }
    public void moveLift(LiftPosition position) {
        moveHeight(position);
        moveRangeOut(position);
    }
    public void moveHeight(LiftPosition position){
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