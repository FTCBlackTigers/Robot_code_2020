package org.firstinspires.ftc.teamcode.robot_systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controller.Controller;

/**
 * This class represents the subsystems that make the robot and contains must have method of each subsystem.
 */
public abstract class SubSystem {
    protected OpMode opMode;

    public abstract void init(HardwareMap hardwareMap, OpMode opMode);

    public abstract void teleopMotion(Controller driver, Controller operator);
}
