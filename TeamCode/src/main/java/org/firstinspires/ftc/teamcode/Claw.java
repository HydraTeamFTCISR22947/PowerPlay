package org.firstinspires.ftc.teamcode;
//Package

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//Imports

public class Claw
{
    private Servo _ClawServo;
    private HardwareMap _hardwaremap;
    private double _openClaw = 0;
    private double _closeClaw = 0;
    //Make change,some changes are with values

    public Claw(HardwareMap hardwaremap)
    {
        this._hardwaremap = hardwaremap;
        _ClawServo = hardwaremap.get(Servo.class,"Claw_servo");
        openClaw();
    }
    //retrieving instances of two Servos from the hardware map and calling openClaw function

    public void openClaw()
    {
        _ClawServo.setPosition(_openClaw);

    }
    //Function that open the Claw

    public void closeClawClaw()
    {
        _ClawServo.setPosition(_closeClaw );
    }
    //Function that close the Claw
}
