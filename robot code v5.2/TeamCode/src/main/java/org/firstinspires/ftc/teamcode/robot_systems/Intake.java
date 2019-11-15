package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;

public class Intake extends SubSystem {
    public enum RampAngle{
        ANGLE_UP(0) , ANGLE_DOWN(140);
        private double angle;
        //TODO: find integer "tickPerDegree"
        private double tickPerDegree = 12.711;
        private RampAngle (double angle){
            this.angle = angle;
        }

        public int getAngleInTicks() {
            return (int) (angle*tickPerDegree);
        }
    }
    DcMotor intakeMotor;
    DcMotor rampAngle;
    Servo gateServo;
    //TODO: ADD TOUCH SENSOR

    private final double rampPower=1;
    private final double power=1;
    private final double upPosRamp=1;
    private final double downPosRamp=0;
    private final double openPosGate=0;
    private final double closePosGate=1;

    @Override
    public void init(HardwareMap hardwareMap, OpMode opMode) {
       rampAngle = hardwareMap.get(DcMotor.class ,"rampAngle");
        intakeMotor =  hardwareMap.get(DcMotor.class , "intakeMotor");
        gateServo= hardwareMap.get(Servo.class, "gateServo");
        this.opMode= opMode;
        gateServo.setDirection(Servo.Direction.FORWARD);
        closeGate();
        rampAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rampAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rampAngle.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void teleopMotion(Controller driver, Controller operator) {
        if (!rampAngle.isBusy()){
            rampAngle.setPower(0);
            rampAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(driver.leftBumper.isPressed()){
            reversIntake();
        }
        else if(driver.rightBumper.isPressed()){
            collectStone();
        }
        if(driver.rightBumper.onRealese() || driver.leftBumper.onRealese()){
            getReadyToCollect();
        }
        if(driver.b.onClick()){
            moveRamp(RampAngle.ANGLE_UP);
        }
        if(driver.a.onClick()){
            moveRamp(RampAngle.ANGLE_DOWN);
        }
        if(operator.a.onClick()){
            openGate();
        }
        if(operator.y.onClick()){
            closeGate();
        }
        opMode.telemetry.addData("Target: ",rampAngle.getTargetPosition());
        opMode.telemetry.addData ("Angle: ",rampAngle.getCurrentPosition());
    }

    public void collectStone(){
        intakeMotor.setPower(power);
    }
    public void reversIntake(){
        intakeMotor.setPower(-power);
    }
    public void openGate(){
        gateServo.setPosition(openPosGate);
    }
    public void moveRamp(RampAngle target) {
        rampAngle.setTargetPosition(target.getAngleInTicks());
        rampAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rampAngle.setPower(Math.abs(rampPower));
    }
    public void closeGate() {
        gateServo.setPosition(closePosGate);
    }
    public void getReadyToCollect(){
        intakeMotor.setPower(0);
        //TODO: MAKE THE METHOD
    }

}
