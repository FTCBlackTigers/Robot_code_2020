package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;

public class MovingStoneArm extends SubSystem{

    public enum ArmAngle{
        START_POSITION(0) , STATE_ONE(0.3) , STATE_TWO(0.6) , STATE_THREE(1) , RELEASE(5);
        private double angle;
        //TODO: find integer "tickPerDegree"
        private double tickPerDegree = 0;
        private ArmAngle (double angle){
            this.angle = angle;
        }

        public int getAngleInTicks() {
            return (int) (angle*tickPerDegree);
        }
    }
    DcMotor motorAngle;
    Servo stoneHold;


    private final double addToAngle = 1;
    private final double GRAB_POS = 1;
    private final double RELEASED_POS = 0;
    private final double motorPower = 1;

    @Override
    public void init(HardwareMap hardwareMap, OpMode opMode) {

        motorAngle = hardwareMap.get(DcMotor.class ,"motorAngle");
        stoneHold = hardwareMap.get(Servo.class ,"stoneHold");
    }

    @Override
    public void teleopMotion(Controller driver, Controller operator) {

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
        motorAngle.setPower(-Math.abs(motorPower));
    }
}

