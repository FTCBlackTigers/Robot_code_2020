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
import java.util.TimerTask;

public class Lift extends SubSystem{
    enum LiftPosition{
        TAKE_STONE(0,0), LEVEL1(7,0.5), LEVEL2(21,0.5), LEVEL3(35,0.5) ,
        LEVEL1_LONG(7,1) , LEVEL2_LONG(21,1) , LEVEL3_LONG(35, 1);

        private double height;
        //TODO CHACK tickperCM
        private double tickPerCM =1;
        private double servoRotation;
        private int servoRotateTime = 1500;
        private LiftPosition (double height, double servoRotation) {
            this.height = height;
            this.servoRotation = servoRotation;
        }
        public int getHeight() {
             return (int) (height*tickPerCM);

        }

        public int getServoRotation() {
            return(int) (servoRotation * servoRotateTime);
        }
    }

    DcMotor liftMotor ;
    Servo liftServo;
    Servo grabServo;
    int servoPosition = 0;
    private final double POWER = 1;
    private final double GRAB_POS = 0.2;
    private final double RELEASED_POS = 0.1;
    private final Timer t = new java.util.Timer();

    LiftPosition[][]positions = {{LiftPosition.LEVEL1,LiftPosition.LEVEL1_LONG},
                                  {LiftPosition.LEVEL2,LiftPosition.LEVEL2_LONG},
                                 {LiftPosition.LEVEL3,LiftPosition.LEVEL3_LONG}};
    private int currenPositionHeight = -1;
    private int currenPositionLengh = -1;

    @Override
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode=opMode;
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftServo = hardwareMap.get(Servo.class, "liftServo");
        grabServo = hardwareMap.get(Servo.class, "grabServo");
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void teleopMotion(Controller driver, Controller operator) {
        if(operator.x.onClick()){
            openGrabServo();
            moveLift(LiftPosition.TAKE_STONE);
            currenPositionHeight=-1;
            currenPositionLengh=-1;
        }
        if(operator.a.onClick()){
            openGrabServo();

        }
        if(operator.b.onClick()){
            closeGrabServo();
        }
        if(operator.dpadUp.onClick()){
            currenPositionHeight++;
            currenPositionHeight= Range.clip(currenPositionHeight,0, positions.length-1);
            currenPositionLengh= Range.clip(currenPositionLengh,0, positions[0].length-1);
            moveLift(positions[currenPositionHeight][currenPositionLengh]);
        }
        if(operator.dpadDown.onClick()){
            currenPositionHeight--;
            currenPositionHeight= Range.clip(currenPositionHeight,0, positions.length-1);
            currenPositionLengh= Range.clip(currenPositionLengh,0, positions[0].length-1);
            moveLift(positions[currenPositionHeight][currenPositionLengh]);
        }
        if(operator.dpadLeft.onClick()){
            currenPositionLengh++;
            currenPositionHeight= Range.clip(currenPositionHeight,0, positions.length-1);
            currenPositionLengh= Range.clip(currenPositionLengh,0, positions[0].length-1);
            moveLift(positions[currenPositionHeight][currenPositionLengh]);
        }
        if(operator.dpadRight.onClick()){
            currenPositionLengh--;
            currenPositionHeight= Range.clip(currenPositionHeight,0, positions.length-1);
            currenPositionLengh= Range.clip(currenPositionLengh,0, positions[0].length-1);
            moveLift(positions[currenPositionHeight][currenPositionLengh]);
        }
        opMode.telemetry.addLine("LIFT");
        opMode.telemetry.addData("\ttarget", liftMotor.getTargetPosition());
        opMode.telemetry.addData("\tcurrent position", liftMotor.getCurrentPosition());
    }
    public void manualTeleop(Controller operator){
        if(operator.a.onClick()){
            openGrabServo();
        }
        if(operator.b.onClick()){
            closeGrabServo();
        }
        if(operator.rightStickY.getValue()>0){
            liftServo.setPosition(1);
        }
        else if(operator.rightStickY.getValue()<0){
            liftServo.setPosition(0);
        }
        else liftServo.setPosition(0.5);
        liftMotor.setPower(operator.leftStickY.getValue());
    }
    public void closeGrabServo(){
        grabServo.setPosition(GRAB_POS);
    }
    public void openGrabServo (){
        grabServo.setPosition(RELEASED_POS);
    }
    public void moveLift(LiftPosition position) {
        if (position.getServoRotation() > servoPosition) {
            servoRotateLeft(position.getServoRotation() - servoPosition);
        }
        else if (position.getServoRotation() < servoPosition) {
            servoRotateRight(  servoPosition - position.getServoRotation());
        }
        liftServo.setPosition(position.getServoRotation());
        liftMotor.setTargetPosition(position.getHeight() - GlobalVariables.endAutoArmEncoder);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(Math.abs(POWER));
        servoPosition = position.getServoRotation();
    }
    public void servoRotateLeft(int time){
        liftServo.setPosition(1);
        servoRotateStop(time);
    }
    public void servoRotateRight(int time){
        liftServo.setPosition(0);
        servoRotateStop(time);
    }
    public void  servoRotateStop(int time){
        t.schedule(new TimerTask() {
            @Override
            public void run() {
                liftServo.setPosition(0.5);
            }
        }, time);
    }
}