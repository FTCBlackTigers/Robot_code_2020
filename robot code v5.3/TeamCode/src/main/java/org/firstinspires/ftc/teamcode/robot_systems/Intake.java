package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.utils.GlobalVariables;

public class Intake extends SubSystem {
    public enum RampAngle {
        ANGLE_UP(95), ANGLE_DOWN(180), UNDER_BRIDGE(180);
        private double angle;

        private double tickPerDegree = 12.711;

        RampAngle(double angle) {
            this.angle = angle;
        }

        public int getAngleInTicks() {
            return (int) (angle * tickPerDegree);
        }
    }

    DcMotor intakeMotor;
    public DcMotor rampAngle;
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

        rampAngle.setDirection(DcMotorSimple.Direction.REVERSE);
        gateServo.setDirection(Servo.Direction.FORWARD);

        rampAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rampAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        closeGate();
    }

    @Override
    public void teleopMotion(Controller driver, Controller operator) {
        if (!rampAngle.isBusy()) {
            rampAngle.setPower(0);
            rampAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        /*if (driver.leftBumper.isPressed()) {
            reversIntake();
        } else if (driver.rightBumper.isPressed()) {
            collectStone();
        }
        if (driver.rightBumper.onRealese() || driver.leftBumper.onRealese()) {
            getReadyToCollect();
        }*/
        if(operator.leftBumper.onClick()){
            moveRamp(RampAngle.ANGLE_DOWN);
            collectStone();
        }
        if (operator.leftBumper.isPressed()) {
            if (rampDistanceSensor.getDistance(DistanceUnit.CM) < 10) {
               moveRamp(RampAngle.ANGLE_UP);
            }
        }
        else if (operator.leftBumper.onRealese()){
            moveRamp(RampAngle.UNDER_BRIDGE);
            getReadyToCollect();
        }
        if (operator.rightStickY.isPressed()){
            rampAngle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rampAngle.setPower(operator.rightStickY.getValue());
        }
        else if (operator.rightStickY.onRealese()){
            rampAngle.setPower(0);
        }
        if (operator.leftTrigger.isPressed()){
            collectStone();
        }
        else if (operator.rightTrigger.isPressed()){
            reversIntake();
        }
        else if (operator.leftTrigger.onRealese() || operator.rightTrigger.onRealese()){
            intakeMotor.setPower(0);
        }
        /*if (driver.b.onClick() || rampDistanceSensor.getDistance(DistanceUnit.CM) < 10) {
            moveRamp(RampAngle.ANGLE_UP);
        }
        if (driver.a.onClick() || stoneArmDistanceSensor.getDistance(DistanceUnit.CM) < 10) {
            moveRamp(RampAngle.ANGLE_DOWN);
        }*/

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
        rampAngle.setTargetPosition(target.getAngleInTicks()- GlobalVariables.endAutoLiftVertical);
        rampAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rampAngle.setPower(Math.abs(RAMP_POWER));
    }

    public void getReadyToCollect() {
        intakeMotor.setPower(0);
        //TODO: MAKE THE METHOD
    }

    public void manualTeleop(Controller driver, Controller operator) {
        rampAngle.setPower(operator.rightStickY.getValue());
        if(operator.x.isPressed()){
            openGate();
        }
        if(operator.y.isPressed()){
            closeGate();
        }
        if(operator.leftTrigger.isPressed()){
            collectStone();
        }
        else if(operator.rightTrigger.isPressed()){
            reversIntake();
        }
        else {
            intakeMotor.setPower(0);
        }
    }
    public void moveRampAuto (RampAngle target){
        moveRamp(target);
        while (Math.abs(rampAngle.getTargetPosition() - rampAngle.getCurrentPosition()) > 50 && ((LinearOpMode)opMode).opModeIsActive()){
            opMode.telemetry.addData ("target", rampAngle.getTargetPosition()).addData("position", rampAngle.getCurrentPosition());
        }
        rampAngle.setPower(0);
        rampAngle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
