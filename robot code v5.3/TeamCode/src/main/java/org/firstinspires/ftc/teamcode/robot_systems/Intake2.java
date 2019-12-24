package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.robot_systems.SubSystem;

public class Intake2<power> extends SubSystem {
    DcMotor leftMotor;
    DcMotor rightMotor;
    final double POWER = 0.5;
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
        if (driver.leftBumper.isPressed()){
            take();
        }
        else if (driver.rightBumper.isPressed()){
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
