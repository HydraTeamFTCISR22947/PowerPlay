package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawServo {

    double openPos = 0; // starting angle of claws
    double closePos = 0.5; // closed angle of claws

    private Servo _clawServo; // declaring claw servo
    private HardwareMap _hardwareMap; // declaring hardware map

    // constructor
    public ClawServo(HardwareMap hardwareMap){

        this._hardwareMap = hardwareMap;
        _clawServo = hardwareMap.get(Servo.class,"claw_servo"); // initialize the claw
        _clawServo.setPosition(openPos); // set open angle

    }
    public void openClaw(){
        // if error occurred , change the order of commands
        _clawServo.setPosition(openPos); // open the claw


    }
    public void closeClaw(){
        // if error, change the order of commands
        _clawServo.setPosition(closePos); // close the claw

    }


}
