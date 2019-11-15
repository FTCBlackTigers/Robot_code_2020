package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;

public class MovingStoneArm extends SubSystem{

    public enum ArmAngle {
        ON_STONE(0), STATE_ONE(0.3), STATE_TWO(0.6), STATE_THREE(1), RELEASE(5) , LOW_POS(6) , HIGH_POS(7);
        private double angle;
        //TODO: find integer "tickPerDegree"
        private double tickPerDegree = 0;

        private ArmAngle(double angle) {
            this.angle = angle;
        }

        public int getAngleInTicks() {
            return (int) (angle * tickPerDegree);
        }
    }

        public enum States{
            AT_TARGET , RELEASE_STONE , MOVING_TO_TARGET , READY_TO_TAKE_STONE,  ;
            }

    DcMotor motorAngle;
    Servo stoneHold;


    private final double addToAngle = 1;
    private final double GRAB_POS = 0;
    private final double RELEASED_POS = 0.6;
    private final double motorPower = 1;
    private final int currentTarget = 0;
    private States currentState = States.AT_TARGET ;

    private final ArmAngle[]choosTarget = {ArmAngle.ON_STONE, ArmAngle.STATE_ONE , ArmAngle.STATE_TWO , ArmAngle.STATE_THREE};


    @Override
    public void init(HardwareMap hardwareMap, OpMode opMode) {

        motorAngle = hardwareMap.get(DcMotor.class ,"motorAngle");
        stoneHold = hardwareMap.get(Servo.class ,"stoneHold");
    }

    @Override
    public void teleopMotion(Controller driver, Controller operator) {

        if(operator.b.onClick()){

        }
        switch (currentState){
            case AT_TARGET:
                break;
            case RELEASE_STONE:
                if (!motorAngle.isBusy()){
                    motorAngle.setPower(0);
                    motorAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    openGrabServo();
                    currentState = States.AT_TARGET;
                }
                break;
            case MOVING_TO_TARGET:
                if (!motorAngle.isBusy() ){
                    motorAngle.setPower(0);
                    motorAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    currentState = States.AT_TARGET;
                }
                break;
            case READY_TO_TAKE_STONE:
                if (!motorAngle.isBusy() ){
                    motorAngle.setPower(0);
                    motorAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (true){
                    //TODO:ask if stone on ramp by sensor
                    moveAngle(ArmAngle.HIGH_POS);

                }
                break;


        }

        if(operator.dpadUp.onClick()){
            //choosTarget[currentTarget + 1];
        }
    }
    public void grabStone (){
     stoneHold.setPosition(GRAB_POS);
    }
    public void openGrabServo (){
     stoneHold.setPosition(RELEASED_POS);
    }
    public void releaseStone (){
       moveAngle ( ArmAngle.RELEASE);
    }


    public void moveAngle(ArmAngle target) {
        if (target == ArmAngle.RELEASE ){
           motorAngle.setTargetPosition (motorAngle.getCurrentPosition() + target.getAngleInTicks());
        }

        else {
            motorAngle.setTargetPosition(target.getAngleInTicks());
        }
        motorAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorAngle.setPower(Math.abs(motorPower));
    }


}

