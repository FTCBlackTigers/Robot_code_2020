package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controller.Controller;

public class Intake extends SubSystem {
    public enum RampAngle {
        ANGLE_UP(0), ANGLE_DOWN(140);
        private double angle;
        //TODO: find integer "tickPerDegree"
        private double tickPerDegree = 12.711;

        RampAngle(double angle) {
            this.angle = angle;
        }

        public int getAngleInTicks() {
            return (int) (angle * tickPerDegree);
        }
    }

    DcMotor intakeMotor;
    DcMotor rampAngle;
    Servo gateServo;
    DistanceSensor rampDistanceSensor;
    DistanceSensor stoneArmDistanceSensor;

    private static final double RAMP_POWER = 1;
    private static final double INTAKE_POWER = 1;
    private static final double OPEN_GATE_POS = 0;
    private static final double CLOSE_GATE_POS = 0.35;

    @Override
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        rampAngle = hardwareMap.get(DcMotor.class, "rampAngle");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        rampDistanceSensor = hardwareMap.get(DistanceSensor.class, "rampDistanceSensor");
        stoneArmDistanceSensor = hardwareMap.get(DistanceSensor.class, "sensor stone");

        gateServo.setDirection(Servo.Direction.FORWARD);

        rampAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rampAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rampAngle.setDirection(DcMotorSimple.Direction.REVERSE);

        closeGate();
    }

    @Override
    public void teleopMotion(Controller driver, Controller operator) {
        if (!rampAngle.isBusy()) {
            rampAngle.setPower(0);
            rampAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (driver.leftBumper.isPressed()) {
            reversIntake();
        } else if (driver.rightBumper.isPressed()) {
            collectStone();
        }
        if (driver.rightBumper.onRealese() || driver.leftBumper.onRealese()) {
            getReadyToCollect();
        }
        if (driver.b.onClick() || rampDistanceSensor.getDistance(DistanceUnit.CM) < 10) {
            moveRamp(RampAngle.ANGLE_UP);
        }
        if (driver.a.onClick() || stoneArmDistanceSensor.getDistance(DistanceUnit.CM) < 10) {
            moveRamp(RampAngle.ANGLE_DOWN);
        }
        if (operator.a.onClick()) {
            openGate();
        }
        if (operator.y.onClick()) {
            closeGate();
        }
        opMode.telemetry.addData("Target: ", rampAngle.getTargetPosition());
        opMode.telemetry.addData("Angle: ", rampAngle.getCurrentPosition());
    }

    public void collectStone() {
        intakeMotor.setPower(INTAKE_POWER);
    }
    public void reversIntake() {
        intakeMotor.setPower(-INTAKE_POWER);
    }

    public void openGate() {
        gateServo.setPosition(OPEN_GATE_POS);
    }
    public void closeGate() {
        gateServo.setPosition(CLOSE_GATE_POS);
    }

    public void moveRamp(RampAngle target) {
        rampAngle.setTargetPosition(target.getAngleInTicks());
        rampAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rampAngle.setPower(Math.abs(RAMP_POWER));
    }

    public void getReadyToCollect() {
        intakeMotor.setPower(0);
        //TODO: MAKE THE METHOD
    }

    public void manualTeleop(Controller driver, Controller operator) {
        if (operator.rightTrigger.isPressed()) {
            rampAngle.setPower(operator.rightTrigger.getValue());

        }
        else if (operator.leftTrigger.isPressed()) {
            rampAngle.setPower(-operator.leftTrigger.getValue());
        }
        else{
            rampAngle.setPower(0);
        }
        if(operator.x.isPressed()){
            openGate();
        }
        if(operator.y.isPressed()){
            closeGate();
        }
        if(driver.rightBumper.isPressed()){
            collectStone();
        }
        if(driver.leftBumper.isPressed()){
            reversIntake();
        }
    }

}
