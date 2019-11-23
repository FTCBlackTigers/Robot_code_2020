package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controller.Controller;

public class MovingStoneArm extends SubSystem{

    public enum ArmAngle {
        ON_STONE(0), LEVEL_ONE(160), LEVEL_TWO(140), LEVEL_THREE(120), RELEASE(10) , LOW_POS(90) , HIGH_POS(120);
        private double angle;
        //TODO: find integer "tickPerDegree"
        private double tickPerDegree = 5;

        private ArmAngle(double angle) {
            this.angle = angle;
        }

        public int getAngleInTicks() {
            return (int) (angle * tickPerDegree);
        }
    }

    public enum States{
        AT_TARGET , RELEASE_STONE , MOVING_TO_TARGET , READY_TO_TAKE_STONE
    }

    DcMotor motorAngle;
    Servo stoneHold;
    DistanceSensor stoneArmDistanceSensor;
    DistanceSensor rampDistanceSensor;

    private final ArmAngle[]choosTarget = {ArmAngle.ON_STONE, ArmAngle.LEVEL_ONE, ArmAngle.LEVEL_TWO, ArmAngle.LEVEL_THREE};

    private static final double addToAngle = 1;
    private static final double GRAB_POS = 0;
    private static final double RELEASED_POS = 0.6;
    private static final double ANGLE_POWER = 1;

    private int currentTarget = 0;
    private States currentState = States.AT_TARGET ;

    @Override
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        motorAngle = hardwareMap.get(DcMotor.class, "motorAngle");
        stoneHold = hardwareMap.get(Servo.class, "stoneHold");
        rampDistanceSensor = hardwareMap.get(DistanceSensor.class, "rampDistanceSensor");
        stoneArmDistanceSensor = hardwareMap.get(DistanceSensor.class, "sensor stone");

        motorAngle.setDirection(DcMotorSimple.Direction.REVERSE);

        motorAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void teleopMotion(Controller driver, Controller operator) {
        if(operator.dpadUp.onClick()){
            currentTarget++;
            currentTarget=Range.clip(currentTarget,1, choosTarget.length-1);
            moveAngle(choosTarget[currentTarget]);
           currentState=States.MOVING_TO_TARGET;

        }
        if(operator.dpadDown.onClick()) {
            currentTarget--;
            currentTarget = Range.clip(currentTarget, 1, choosTarget.length - 1);
            moveAngle(choosTarget[currentTarget]);
            currentState = States.MOVING_TO_TARGET;
        }
        if(operator.b.onClick()){
            moveAngle(ArmAngle.RELEASE);
            currentState= States.RELEASE_STONE;
        }
        if(operator.x.onClick()){
            moveAngle(ArmAngle.LOW_POS);
            currentState= States.READY_TO_TAKE_STONE;
        }
        opMode.telemetry.addData("current State",currentState);
        opMode.telemetry.addData("target",motorAngle.getTargetPosition());
        opMode.telemetry.addData("current position",motorAngle.getCurrentPosition());
        opMode.telemetry.addData("servo position",stoneHold.getPosition());
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
                if (driver.b.onClick() || rampDistanceSensor.getDistance(DistanceUnit.CM)<10){
                    moveAngle(ArmAngle.HIGH_POS);

                }
                if(stoneArmDistanceSensor.getDistance(DistanceUnit.CM)<10){
                    openGrabServo();
                    moveAngle(ArmAngle.ON_STONE);
                    if(!motorAngle.isBusy()){
                        grabStone();
                        currentState = States.AT_TARGET;
                    }
                }
                break;
        }
    }
    public void grabStone (){
     stoneHold.setPosition(GRAB_POS);
    }
    public void openGrabServo (){
     stoneHold.setPosition(RELEASED_POS);
    }

    public void moveAngle(ArmAngle target) {
        if (target == ArmAngle.RELEASE ){
           motorAngle.setTargetPosition (motorAngle.getCurrentPosition() + target.getAngleInTicks());
        }

        else {
            motorAngle.setTargetPosition(target.getAngleInTicks());
        }
        motorAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorAngle.setPower(Math.abs(ANGLE_POWER));
    }

    public void manualTeleop(Controller driver, Controller operator){
        if(operator.a.isPressed()){
            motorAngle.setPower(-0.5);
        }
        else if (operator.b.isPressed()){
            motorAngle.setPower(0.5);
        }
        else{
            motorAngle.setPower(0);
        }
    }
}

