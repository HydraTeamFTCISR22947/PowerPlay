package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawServo {

    public static double closePos = 0; // starting angle of claws
    public static double openPos = 0.2; // closed angle of claws

    private Servo _clawServo; // declaring claw servo
    private HardwareMap _hardwareMap; // declaring hardware map

    // constructor
    //lil mish
    public ClawServo(HardwareMap hardwareMap){

        this._hardwareMap = hardwareMap;
        _clawServo = hardwareMap.get(Servo.class,"claw_servo"); // initialize the claw
        openClaw();
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
