package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;

public class Intake extends SubSystem {
    DcMotor intake;
    Servo rightServo;
    Servo leftServo;
    Servo gateServo;
    //TODO: ADD TOUCH SENSOR

    private final double power=0.5;
    private final double upPosRamp=1;
    private final double downPosRamp=0;
    private final double openPosGate=0;
    private final double closePosGate=1;

    @Override
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        intake= hardwareMap.get(DcMotor.class , "intakeMotor");
        leftServo= hardwareMap.get(Servo.class , "leftServo");
        rightServo= hardwareMap.get(Servo.class , "rightServo");
        gateServo= hardwareMap.get(Servo.class, "gateServo");
        this.opMode= opMode;
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.FORWARD);
        gateServo.setDirection(Servo.Direction.FORWARD);

        liftRamp();
        closeGate();
    }

    @Override
    public void teleopMotion(Controller driver, Controller operator) {

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
            liftRamp();
        }
        if(driver.a.onClick()){
            pullDownRamp();
        }
        if(operator.a.onClick()){
            openGate();
        }
        if(operator.y.onClick()){
            closeGate();
        }
    }

    public void collectStone(){
        intake.setPower(power);
    }
    public void reversIntake(){
        intake.setPower(-power);
    }
    public void liftRamp(){
        leftServo.setPosition(upPosRamp);
        rightServo.setPosition(upPosRamp);
    }
    public void pullDownRamp(){
        leftServo.setPosition(downPosRamp);
        rightServo.setPosition(downPosRamp);
    }
    public void openGate(){
        gateServo.setPosition(openPosGate);
    }

    public void closeGate() {
        gateServo.setPosition(closePosGate);
    }
    public void getReadyToCollect(){
        intake.setPower(0);
        //TODO: MAKE THE METHOD
    }

}
