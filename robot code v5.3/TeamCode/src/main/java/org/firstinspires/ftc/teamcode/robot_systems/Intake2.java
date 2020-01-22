package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.robot_systems.SubSystem;

public class Intake2 extends SubSystem {
   private DcMotor leftMotor;
   private DcMotor rightMotor;
   private static final double POWER = 1;
    @Override
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;

        leftMotor = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightIntakeMotor");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void teleopMotion(Controller driver, Controller operator) {
        if (operator.leftTrigger.isPressed()){
            take();
        }
        else if (operator.rightTrigger.isPressed()){
            reverse();
        }
        else{
            stop();
        }
    }
    public void take(){
        leftMotor.setPower(POWER);
        rightMotor.setPower(POWER);
    }
    public void reverse(){
        leftMotor.setPower(-POWER);
        rightMotor.setPower(-POWER);
    }
    public void stop(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}

