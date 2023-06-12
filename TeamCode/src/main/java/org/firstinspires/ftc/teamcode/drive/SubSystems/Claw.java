package org.firstinspires.ftc.teamcode.drive.SubSystems;
//Package

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//Imports

public class Claw extends SubsystemBase
{
    private Servo _ClawServoLeft;
    private Servo _ClawServoRight;
    private double _openClaw = 0.25;
    private double _closeClaw = 0.06;
    //Make change,some changes are with values

    public Claw(HardwareMap hardwaremap)
    {
        _ClawServoLeft = hardwaremap.get(Servo.class,"claw_servo_left");
        _ClawServoRight = hardwaremap.get(Servo.class,"claw_servo_right");

        _ClawServoRight.setDirection(Servo.Direction.REVERSE);

        openClaw();
    }
    //retrieving instances of two Servos from the hardware map and calling openClaw function

    public void openClaw()
    {
        _ClawServoLeft.setPosition(_openClaw);
        _ClawServoRight.setPosition(_openClaw);

    }
    //Function that open the Claw

    public void closeClawClaw()
    {
        _ClawServoLeft.setPosition(_closeClaw );
        _ClawServoRight.setPosition(_closeClaw );
    }
    //Function that close the Claw
}

